#ifndef __ASMARM_ARCH_SCU_H
#define __ASMARM_ARCH_SCU_H

unsigned int scu_get_core_count(void __iomem *);
void scu_inv_cpu(void __iomem *, int cpu);
void scu_enable(void __iomem *);

#endif
