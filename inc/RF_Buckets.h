/*
 * RF_Buckets.h
 *
 *  Created on: 18 mars 2018
 *      Author: erik
 */

#ifndef RF_BUCKETS_H_
#define RF_BUCKETS_H_

extern void SendRFBuckets(uint16_t *buckets, uint8_t *rfdata, uint8_t n, uint8_t repeats);
extern void Bucket_Received(uint16_t duration);
extern void SendRF_Sync(void);

#endif /* RF_BUCKETS_H_ */
