#define _DEFAULT_SOURCE
#include <stdio.h>
#include <unistd.h>
#include <omp.h>


int NITER=100;
int N=100;

int main(){
  for(int i = 0; i < NITER; ++i){
    float counter = 0.0;
    #pragma omp parallel for
    for(int j = 0; j < N; ++j){
      // Useful work here
      counter = counter + 1;
      usleep(1);
    }
  }
}
