/**
  You can confirm affinity by $ taskset -apc pid
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <sched.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>

void *thread_main(void *arg)
{
  int cpu = *(int *)arg;
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  CPU_SET(cpu, &cpu_set);
  
  int result = sched_setaffinity(0, sizeof(cpu_set_t), &cpu_set);
  if (result != 0) {
    perror("sched_setaffinity():");
    return NULL;
  }

  int c=0;
  while(1){
    if (c%1000000) printf("running on %d\n", cpu);
    c++;
  }

  
}

int main() {
  #if 1
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  CPU_SET(0, &cpu_set);
  
  int result = sched_setaffinity(0, sizeof(cpu_set_t), &cpu_set);
  if (result != 0) {
    perror("sched_setaffinity():");
  }
#endif
  
  pthread_t thr1, thr2;
  int cpu1=1, cpu2=2;
  pthread_create(&thr1, NULL, thread_main, &cpu1);
  pthread_create(&thr2, NULL, thread_main, &cpu2);

  pthread_join(thr1, NULL);
  pthread_join(thr2, NULL);
  return 0;
}
