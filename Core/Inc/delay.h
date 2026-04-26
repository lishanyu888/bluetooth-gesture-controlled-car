#ifndef CAR_DELAY_H
#define CAR_DELAY_H

#include <stdint.h>

void Delay_Init(void);
void Delay_Us(uint32_t us);
void Delay_Ms(uint32_t ms);

#endif /* CAR_DELAY_H */
