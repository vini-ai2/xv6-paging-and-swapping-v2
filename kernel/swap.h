// kernel/swap.h
#ifndef SWAP_H
#define SWAP_H

// Helper function declarations
int add_swap_slot(struct proc *p, int slot);
int get_swap_slot_for_va(struct proc *p, uint64 va);
void remove_swap_slot(struct proc *p, int slot);
int is_swapped_out(pte_t *pte);
void mark_swapped_out(pagetable_t pagetable, uint64 va, int swap_slot);
int get_swap_slot_from_pte(pte_t *pte);

#endif