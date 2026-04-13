// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.
#include "types.h"
#include "memlayout.h"
#include "param.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"



// Hardware Global Core Map details
struct frame {
  int state;          // 0=free, 1=pinned(kernel/guard), 2=user(pageable)
  struct proc *owner; // Pointer to owning process
  uint64 va;          // Virtual Address mapped at
  int next;           // Index to next frame in process's reverse map list
  int prev;           // Index to prev frame
};

struct frame frame_table[PHYSTOP / PGSIZE];
struct spinlock ft_lock;
int clock_hand = 0;

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;

void kinit() {
  initlock(&kmem.lock, "kmem");
  initlock(&ft_lock, "ft_lock");
  for (int i = 0; i < PHYSTOP / PGSIZE; i++) {
    frame_table[i].state = 1; // Mark strictly as pinned initially
    frame_table[i].owner = 0;
    frame_table[i].va = 0;
    frame_table[i].next = -1;
    frame_table[i].prev = -1;
  }
  freerange(end, (void *)PHYSTOP);
}

void freerange(void *pa_start, void *pa_end) {
  char *p;
  p = (char *)PGROUNDUP((uint64)pa_start);
  for (; p + PGSIZE <= (char *)pa_end; p += PGSIZE)
    kfree(p);
}

// Free the page of physical memory pointed at by pa,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void kfree(void *pa) {
  struct run *r;

  if (((uint64)pa % PGSIZE) != 0 || (char *)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  int idx = (uint64)pa / PGSIZE;
  acquire(&ft_lock);
  if (frame_table[idx].state == 2 && frame_table[idx].owner) {
    struct proc *p = frame_table[idx].owner;
    // Sever per-process reverse map list ties securely
    if (frame_table[idx].prev != -1) {
      frame_table[frame_table[idx].prev].next = frame_table[idx].next;
    } else {
      p->rmap_head = frame_table[idx].next;
    }
    if (frame_table[idx].next != -1) {
      frame_table[frame_table[idx].next].prev = frame_table[idx].prev;
    }
  }

  // Wipe tracking state
  frame_table[idx].state = 0;
  frame_table[idx].owner = 0;
  frame_table[idx].va = 0;
  frame_table[idx].next = -1;
  frame_table[idx].prev = -1;
  release(&ft_lock);

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  r = (struct run *)pa;

  acquire(&kmem.lock);
  r->next = kmem.freelist;
  kmem.freelist = r;
  release(&kmem.lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void *kalloc(void) {
  struct run *r;

  acquire(&kmem.lock);
  r = kmem.freelist;
  if (r)
    kmem.freelist = r->next;
  release(&kmem.lock);

  if (r) {
    memset((char *)r, 5, PGSIZE); // fill with junk
    int idx = (uint64)r / PGSIZE;
    acquire(&ft_lock);
    // Secure by default: Any page kalloc() grants starts as pinned/kernel
    // memory
    frame_table[idx].state = 1;
    frame_table[idx].owner = 0;
    frame_table[idx].va = 0;
    frame_table[idx].next = -1;
    frame_table[idx].prev = -1;
    release(&ft_lock);
  }
  return (void *)r;
}

// Register a previously pinned/kernel page as user memory.
// Attaches the frame to the process's internal reverse map.
void kalloc_user_map(uint64 pa, struct proc *p, uint64 va) {
  if (!p)
    return;
  int idx = pa / PGSIZE;
  acquire(&ft_lock);

  frame_table[idx].state = 2; // Demote from pinned to pageable user state
  frame_table[idx].owner = p;
  frame_table[idx].va = va;

  // Insert at the head of the process's O(1) reverse map list
  frame_table[idx].next = p->rmap_head;
  frame_table[idx].prev = -1;
  if (p->rmap_head != -1) {
    frame_table[p->rmap_head].prev = idx;
  }
  p->rmap_head = idx;

  release(&ft_lock);
}

// Memory Victim Selection: Clock Page Replacement Algorithm
// Returns the physical address of the victimized frame, or 0 if exhausted.
uint64 pick_victim(void) {
  acquire(&ft_lock);

  // Two passes: First pass tries to locate and zero A-bits.
  // Second pass guarantees collection of cleared frames.
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 0; i < (PHYSTOP / PGSIZE); i++) {
      int idx = clock_hand;
      struct frame *f = &frame_table[idx];

      // Exclude pinned(1), free(0), and broken frames
      if (f->state == 2 && f->owner) {
        pte_t *pte = walk(f->owner->pagetable, f->va, 0);
        if (pte && (*pte & PTE_V)) {
          if (*pte & PTE_A) {
            *pte &= ~PTE_A; // hardware access detected, clear logic bit (2nd
                            // chance)
          } else {
            // Victim Selected safely
            clock_hand = (clock_hand + 1) % (PHYSTOP / PGSIZE);
            release(&ft_lock);
            return idx * PGSIZE;
          }
        }
      }
      clock_hand = (clock_hand + 1) % (PHYSTOP / PGSIZE);
    }
  }

  release(&ft_lock);
  return 0; // Absolute OOM protection
}
