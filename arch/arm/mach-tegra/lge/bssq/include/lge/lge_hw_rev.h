#ifndef LGE_HWREV_H
#define LGE_HWREV_H

typedef enum pcb_rev {
   REV_A,
   REV_B,
   REV_C,
   REV_D,
   REV_E,
   REV_F,
   REV_10,
   REV_11,
   REV_MAX,
}hw_rev_t;

extern hw_rev_t get_lge_pcb_revision(void);
#endif /* LGE_HWREV_H */
