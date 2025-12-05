
#include <unistd.h>
#include <string.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"

int absoluteUsedMemory = 0;
int usedMemory = 0;

void * microros_allocate(size_t size, void * state){
  (void) state;
  absoluteUsedMemory += size;
  usedMemory += size;
  return pvPortMalloc(size);
}

void microros_deallocate(void * pointer, void * state){
  (void) state;
  if (NULL != pointer){
    vPortFree(pointer);
  }
}

void * microros_reallocate(void * pointer, size_t size, void * state){
  (void) state;
  if (NULL == pointer){
    return pvPortMalloc(size);
  } else {
    void * new_ptr = pvPortMalloc(size);
    if (new_ptr != NULL && pointer != NULL) {
      // Copy old data (we don't know the old size, so use size as estimate)
      memcpy(new_ptr, pointer, size);
      vPortFree(pointer);
    }
    return new_ptr;
  }
}

void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state){
  (void) state;
  size_t total_size = number_of_elements * size_of_element;
  absoluteUsedMemory += total_size;
  usedMemory += total_size;
  void * ptr = pvPortMalloc(total_size);
  if (ptr != NULL) {
    memset(ptr, 0, total_size);
  }
  return ptr;
}