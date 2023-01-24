#include "Semaphores.h"
#include <hal.h>
#include "AP_HAL_rp2040ChibiOS.h"

#if CH_CFG_USE_MUTEXES == TRUE

extern const AP_HAL::HAL& hal;

using namespace Rp2040ChibiOS;

// constructor
Semaphore::Semaphore()
{
    static_assert(sizeof(_lock) >= sizeof(mutex_t), "invalid mutex size");
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxObjectInit(mtx);
}

bool Semaphore::give() {
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxUnlock(mtx);
    return true;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    mutex_t *mtx = (mutex_t *)_lock;
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        chMtxLock(mtx);
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

bool Semaphore::take_nonblocking()
{
    mutex_t *mtx = (mutex_t *)_lock;
    return chMtxTryLock(mtx);
}

bool Semaphore::check_owner(void)
{
    mutex_t *mtx = (mutex_t *)_lock;
    return mtx->owner == chThdGetSelfX();
}

void Semaphore::assert_owner(void)
{
    osalDbgAssert(check_owner(), "owner");
}

#endif // CH_CFG_USE_MUTEXES
