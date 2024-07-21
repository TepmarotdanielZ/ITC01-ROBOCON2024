/*
 * IIRFilter.h
 *
 *  Created on: Aug 9, 2022
 *      Author: CHUOB VANDY
 */

#ifndef INC_IIRFILTER_H_
#define INC_IIRFILTER_H_

typedef struct {
	float alpha;
	float out;
}IIRFilter;

void IIRFilter_Init(IIRFilter *filter, float alpha){
	filter->alpha = alpha;
	filter->out	  = 0.0f;
}
float IIRFilter_Update(IIRFilter *filter, float in){
	filter->out = filter->alpha * filter->out + ( 1.0f - filter->alpha ) * in;
	return filter->out;
}

#endif /* INC_IIRFILTER_H_ */
