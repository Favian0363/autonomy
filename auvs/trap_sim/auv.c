// Deliberately faulty plugin used only for explicit crash-isolation experiments.
#include "../../include/auv.h"
void auv_init(void) {
  __builtin_trap();
}
void auv_yield_until_next_frame(AuvFrame *frame) {
  (void)frame;
  __builtin_trap();
}
void auv_deinit(void) { }
