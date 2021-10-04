#ifndef TINBUSERROR_H
#define TINBUSERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#define TinBus_kOK (0)
#define TinBus_kWriteBusy (1)
#define TinBus_kWriteComplete (2)
#define TinBus_kWriteCollision (3)
#define TinBus_kWriteTimeout (4)
#define TinBus_kReadCRCError (5)
#define TinBus_kReadOverunError (6)

#ifdef __cplusplus
} // extern "C"
#endif

#endif // TINBUSERROR_H
