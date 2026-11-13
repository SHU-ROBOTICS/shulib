// n02_misspelt_preset: a misspelt preset (GCC cites this line in its expansion note; the error itself is in kinds.hpp).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::arm["scroe"];
// expect: no_such_preset_on_this_mechanism
// expect-at: 4
