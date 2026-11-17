<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/driver_inputs.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `driver_inputs.hpp`

mechanism/driver_inputs.hpp — THE DRIVER FILE'S MECHANISM CONTROLS: hold-to-run, toggle, preset and the homing controls, each naming its mechanism by its DECLARATION, checked at build against the robot's list, and run THROUGH the mechanism service.

This header declares **4** types (37 members), **51** free functions, **1** constant, and **8** type aliass.

Extracted from [`include/shulib/mechanism/driver_inputs.hpp`](../../include/shulib/mechanism/driver_inputs.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kToggleAtSpeedWindowSeconds`](#ktoggleatspeedwindowseconds) — *constant*
- [`enum class ControlKind`](#enum-class-controlkind)
  - [`Hold`](#controlkind-hold)
  - [`Toggle`](#controlkind-toggle)
  - [`Preset`](#controlkind-preset)
  - [`HomeOnPress`](#controlkind-homeonpress)
  - [`HomeAtDriverStart`](#controlkind-homeatdriverstart)
  - [`AutonOnly`](#controlkind-autononly)
- [`struct Control`](#struct-control)
  - [`kind`](#control-kind)
  - [`mechanismKind`](#control-mechanismkind)
  - [`mechanism`](#control-mechanism)
  - [`mechanismName`](#control-mechanismname)
  - [`button`](#control-button)
  - [`back`](#control-back)
  - [`volts`](#control-volts)
  - [`bound`](#control-bound)
  - [`value`](#control-value)
  - [`position`](#control-position)
  - [`presetName`](#control-presetname)
  - [`units`](#control-units)
  - [`oneWay`](#control-oneway)
- [`whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset`](#whileheld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset) — *free function*
- [`whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two`](#whileheld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two) — *free function*
- [`a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot`](#a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot) — *free function*
- [`a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed`](#a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed) — *free function*
- [`homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift`](#homeonpress_and_homeatdriverstart_are_for_an_arm_or_a_lift) — *free function*
- [`a_control_names_a_mechanism_this_robot_does_not_have`](#a_control_names_a_mechanism_this_robot_does_not_have) — *free function*
- [`a_control_names_a_button_that_is_not_one_of_the_twelve`](#a_control_names_a_button_that_is_not_one_of_the_twelve) — *free function*
- [`button_L1_does_two_jobs`](#button_l1_does_two_jobs) — *free function*
- [`button_L2_does_two_jobs`](#button_l2_does_two_jobs) — *free function*
- [`button_R1_does_two_jobs`](#button_r1_does_two_jobs) — *free function*
- [`button_R2_does_two_jobs`](#button_r2_does_two_jobs) — *free function*
- [`button_Up_does_two_jobs`](#button_up_does_two_jobs) — *free function*
- [`button_Down_does_two_jobs`](#button_down_does_two_jobs) — *free function*
- [`button_Left_does_two_jobs`](#button_left_does_two_jobs) — *free function*
- [`button_Right_does_two_jobs`](#button_right_does_two_jobs) — *free function*
- [`button_X_does_two_jobs`](#button_x_does_two_jobs) — *free function*
- [`button_B_does_two_jobs`](#button_b_does_two_jobs) — *free function*
- [`button_Y_does_two_jobs`](#button_y_does_two_jobs) — *free function*
- [`button_A_does_two_jobs`](#button_a_does_two_jobs) — *free function*
- [`a_hold_needs_volts_above_0_and_at_most_12`](#a_hold_needs_volts_above_0_and_at_most_12) — *free function*
- [`a_preset_or_a_homing_needs_a_time_limit_above_zero`](#a_preset_or_a_homing_needs_a_time_limit_above_zero) — *free function*
- [`two_controls_of_the_same_kind_on_one_mechanism`](#two_controls_of_the_same_kind_on_one_mechanism) — *free function*
- [`a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed`](#a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed) — *free function*
- [`a_homing_control_on_a_mechanism_that_is_never_homed`](#a_homing_control_on_a_mechanism_that_is_never_homed) — *free function*
- [`a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant`](#a_mechanism_has_no_driver_control_mark_it_autononly_if_that_is_meant) — *free function*
- [`a_mechanism_marked_autonOnly_also_has_a_control`](#a_mechanism_marked_autononly_also_has_a_control) — *free function*
- [`more_controls_than_fit`](#more_controls_than_fit) — *free function*
- [`a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control`](#a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control) — *free function*
- [`a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it`](#a_never_homed_mechanism_needs_unhomedvolts_for_its_buttons_to_move_it) — *free function*
- [`a_preset_on_a_mechanism_that_is_never_homed`](#a_preset_on_a_mechanism_that_is_never_homed) — *free function*
- [`whileHeld`](#whileheld) — *free function*
- [`whileHeld (overload 2)`](#whileheld-2) — *free function*
- [`whileHeld (overload 3)`](#whileheld-3) — *free function*
- [`whileHeld (overload 4)`](#whileheld-4) — *free function*
- [`whileHeld (overload 5)`](#whileheld-5) — *free function*
- [`whileHeld (overload 6)`](#whileheld-6) — *free function*
- [`toggle`](#toggle) — *free function*
- [`toggle (overload 2)`](#toggle-2) — *free function*
- [`toggle (overload 3)`](#toggle-3) — *free function*
- [`toggle (overload 4)`](#toggle-4) — *free function*
- [`preset`](#preset) — *free function*
- [`preset (overload 2)`](#preset-2) — *free function*
- [`preset (overload 3)`](#preset-3) — *free function*
- [`homeOnPress`](#homeonpress) — *free function*
- [`homeOnPress (overload 2)`](#homeonpress-2) — *free function*
- [`homeAtDriverStart`](#homeatdriverstart) — *free function*
- [`homeAtDriverStart (overload 2)`](#homeatdriverstart-2) — *free function*
- [`homeOnPress (overload 3)`](#homeonpress-3) — *free function*
- [`homeAtDriverStart (overload 3)`](#homeatdriverstart-3) — *free function*
- [`autonOnly`](#autononly) — *free function*
- [`struct DriverControls`](#struct-drivercontrols)
  - [`kCap`](#drivercontrols-kcap)
  - [`c`](#drivercontrols-c)
  - [`count`](#drivercontrols-count)
  - [`all`](#drivercontrols-all)
  - [`avoids`](#drivercontrols-avoids)
- [`controls`](#controls) — *free function*
- [`class DriverInputs`](#class-driverinputs)
  - [`kCap`](#driverinputs-kcap)
  - [`DriverInputs`](#driverinputs-driverinputs)
  - [`DriverInputs (overload 2)`](#driverinputs-driverinputs-2)
  - [`operator=`](#driverinputs-operator-eq)
  - [`DriverInputs (overload 3)`](#driverinputs-driverinputs-3)
  - [`operator= (overload 2)`](#driverinputs-operator-eq-2)
  - [`~DriverInputs`](#driverinputs-destructor-driverinputs)
  - [`bind`](#driverinputs-bind)
  - [`avoids`](#driverinputs-avoids)
  - [`bound`](#driverinputs-bound)
  - [`boundTo`](#driverinputs-boundto)
  - [`enter`](#driverinputs-enter)
  - [`tick`](#driverinputs-tick)
- [`DriverControls`](#drivercontrols) — *type alias*
- [`controls (overload 2)`](#controls-2) — *type alias*
- [`whileHeld (overload 7)`](#whileheld-7) — *type alias*
- [`toggle (overload 5)`](#toggle-5) — *type alias*
- [`preset (overload 4)`](#preset-4) — *type alias*
- [`homeOnPress (overload 4)`](#homeonpress-4) — *type alias*
- [`homeAtDriverStart (overload 4)`](#homeatdriverstart-4) — *type alias*
- [`autonOnly (overload 2)`](#autononly-2) — *type alias*

<a id="ktoggleatspeedwindowseconds"></a>

## `kToggleAtSpeedWindowSeconds`

```cpp
inline constexpr double kToggleAtSpeedWindowSeconds = 5.0
```

A driver toggle's flywheel spin-up: its "at speed" verdict window, seconds (HA-182 — invented, PROVISIONAL; a LIBRARY constant). Not a match number: the wheel keeps spinning whatever the verdict; it only bounds the step the toggle logs.

*constant, declared at [`include/shulib/mechanism/driver_inputs.hpp:102`](../../include/shulib/mechanism/driver_inputs.hpp#L102).*

<a id="enum-class-controlkind"></a>

## `enum class ControlKind`

```cpp
enum class ControlKind : std::uint8_t
```

What one driver control does (file header).

*enum class, declared at [`include/shulib/mechanism/driver_inputs.hpp:107`](../../include/shulib/mechanism/driver_inputs.hpp#L107).*

<a id="controlkind-hold"></a>

### `ControlKind::Hold`

```cpp
Hold
```

whileHeld: two buttons ± volts, or one button + volts on a power one (oneWay)

*enumerator, declared at [`include/shulib/mechanism/driver_inputs.hpp:108`](../../include/shulib/mechanism/driver_inputs.hpp#L108).*

<a id="controlkind-toggle"></a>

### `ControlKind::Toggle`

```cpp
Toggle
```

one button: flips a two-position pneumatic, or a flywheel's named speed on / off

*enumerator, declared at [`include/shulib/mechanism/driver_inputs.hpp:109`](../../include/shulib/mechanism/driver_inputs.hpp#L109).*

<a id="controlkind-preset"></a>

### `ControlKind::Preset`

```cpp
Preset
```

one button: a position, a speed or a pneumatic position

*enumerator, declared at [`include/shulib/mechanism/driver_inputs.hpp:110`](../../include/shulib/mechanism/driver_inputs.hpp#L110).*

<a id="controlkind-homeonpress"></a>

### `ControlKind::HomeOnPress`

```cpp
HomeOnPress
```

one button: home it by its declaration's method

*enumerator, declared at [`include/shulib/mechanism/driver_inputs.hpp:111`](../../include/shulib/mechanism/driver_inputs.hpp#L111).*

<a id="controlkind-homeatdriverstart"></a>

### `ControlKind::HomeAtDriverStart`

```cpp
HomeAtDriverStart
```

no button: home it once at driver-control entry, when nothing homed it

*enumerator, declared at [`include/shulib/mechanism/driver_inputs.hpp:112`](../../include/shulib/mechanism/driver_inputs.hpp#L112).*

<a id="controlkind-autononly"></a>

### `ControlKind::AutonOnly`

```cpp
AutonOnly
```

no control, on purpose

*enumerator, declared at [`include/shulib/mechanism/driver_inputs.hpp:113`](../../include/shulib/mechanism/driver_inputs.hpp#L113).*

<a id="struct-control"></a>

## `struct Control`

```cpp
struct Control
```

One driver-file line. Made only by the words below, each taking the mechanism's DECLARATION (or a goal it made), so a control always names its mechanism.

*struct, declared at [`include/shulib/mechanism/driver_inputs.hpp:118`](../../include/shulib/mechanism/driver_inputs.hpp#L118).*

<a id="control-kind"></a>

### `Control::kind`

```cpp
ControlKind kind = ControlKind::Hold
```

What it does.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:120`](../../include/shulib/mechanism/driver_inputs.hpp#L120).*

<a id="control-mechanismkind"></a>

### `Control::mechanismKind`

```cpp
Kind mechanismKind = Kind::Power
```

The kind of mechanism it commands (a declaration's kind).

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:122`](../../include/shulib/mechanism/driver_inputs.hpp#L122).*

<a id="control-mechanism"></a>

### `Control::mechanism`

```cpp
const void* mechanism = nullptr
```

The declaration's address — what controls() checks against the robot's list.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:124`](../../include/shulib/mechanism/driver_inputs.hpp#L124).*

<a id="control-mechanismname"></a>

### `Control::mechanismName`

```cpp
const char* mechanismName = nullptr
```

The declaration's `.name` — what DriverInputs::bind() finds its slot by.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:126`](../../include/shulib/mechanism/driver_inputs.hpp#L126).*

<a id="control-button"></a>

### `Control::button`

```cpp
hal::ControllerButton button = hal::ControllerButton::A
```

A hold's FORWARD (up) button; every other control's one button.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:128`](../../include/shulib/mechanism/driver_inputs.hpp#L128).*

<a id="control-back"></a>

### `Control::back`

```cpp
hal::ControllerButton back = hal::ControllerButton::A
```

A hold's BACK (down) button (a ONE-button hold: its one button again — `oneWay` says so).

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:130`](../../include/shulib/mechanism/driver_inputs.hpp#L130).*

<a id="control-volts"></a>

### `Control::volts`

```cpp
double volts = 0.0
```

A hold's volts, (0, 12].

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:132`](../../include/shulib/mechanism/driver_inputs.hpp#L132).*

<a id="control-bound"></a>

### `Control::bound`

```cpp
double bound = 0.0
```

A preset's or a homing's time limit, seconds (P4: every action bounded).

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:134`](../../include/shulib/mechanism/driver_inputs.hpp#L134).*

<a id="control-value"></a>

### `Control::value`

```cpp
double value = 0.0
```

A preset's target: degrees, inches or output rpm (a flywheel toggle's speed too).

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:136`](../../include/shulib/mechanism/driver_inputs.hpp#L136).*

<a id="control-position"></a>

### `Control::position`

```cpp
std::uint8_t position = 0
```

A pneumatic preset's position (its index in the declaration).

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:138`](../../include/shulib/mechanism/driver_inputs.hpp#L138).*

<a id="control-presetname"></a>

### `Control::presetName`

```cpp
const char* presetName = nullptr
```

The preset's (or a pneumatic position's) name; nullptr for a constant target.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:140`](../../include/shulib/mechanism/driver_inputs.hpp#L140).*

<a id="control-units"></a>

### `Control::units`

```cpp
Units units = static_cast<Units>(0)
```

An arm's (degrees) or a lift's (inches) control: its units; UNSET for the other kinds.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:142`](../../include/shulib/mechanism/driver_inputs.hpp#L142).*

<a id="control-oneway"></a>

### `Control::oneWay`

```cpp
bool oneWay = false
```

A ONE-button hold on a POWER mechanism (`whileHeld(m, button, volts)`, chunk ROB1-C2): `button` held = +volts, released = stop; it has no back button (`back` repeats `button` and is never read).

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:145`](../../include/shulib/mechanism/driver_inputs.hpp#L145).*

<a id="whileheld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset"></a>

## `whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset`

```cpp
inline void whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset()
```

whileHeld(...) on a pneumatic or a flywheel (n24).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:150`](../../include/shulib/mechanism/driver_inputs.hpp#L150).*

<a id="whileheld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two"></a>

## `whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two`

```cpp
inline void whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two()
```

whileHeld(m, button, volts) — ONE button — on an arm, a lift, a pneumatic or a flywheel (ROB1-C2).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:152`](../../include/shulib/mechanism/driver_inputs.hpp#L152).*

<a id="a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot"></a>

## `a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot`

```cpp
inline void a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot()
```

toggle(flywheel, ...) without its speed (n51).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:154`](../../include/shulib/mechanism/driver_inputs.hpp#L154).*

<a id="a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed"></a>

## `a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed`

```cpp
inline void a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed()
```

toggle(...) on a power mechanism, an arm or a lift.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:156`](../../include/shulib/mechanism/driver_inputs.hpp#L156).*

<a id="homeonpress_and_homeatdriverstart_are_for_an_arm_or_a_lift"></a>

## `homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift`

```cpp
inline void homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift()
```

homeOnPress(...) / homeAtDriverStart(...) on a power mechanism, a flywheel or a pneumatic.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:158`](../../include/shulib/mechanism/driver_inputs.hpp#L158).*

<a id="a_control_names_a_mechanism_this_robot_does_not_have"></a>

## `a_control_names_a_mechanism_this_robot_does_not_have`

```cpp
inline void a_control_names_a_mechanism_this_robot_does_not_have()
```

A control names a mechanism the robot's list does not have (n19).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:160`](../../include/shulib/mechanism/driver_inputs.hpp#L160).*

<a id="a_control_names_a_button_that_is_not_one_of_the_twelve"></a>

## `a_control_names_a_button_that_is_not_one_of_the_twelve`

```cpp
inline void a_control_names_a_button_that_is_not_one_of_the_twelve()
```

A control names a button that is not one of the controller's twelve (a cast byte).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:162`](../../include/shulib/mechanism/driver_inputs.hpp#L162).*

<a id="button_l1_does_two_jobs"></a>

## `button_L1_does_two_jobs`

```cpp
inline void button_L1_does_two_jobs()
```

L1 does two jobs (a button used twice NAMES the button; the error points at the controls( line).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:164`](../../include/shulib/mechanism/driver_inputs.hpp#L164).*

<a id="button_l2_does_two_jobs"></a>

## `button_L2_does_two_jobs`

```cpp
inline void button_L2_does_two_jobs()
```

L2 does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:166`](../../include/shulib/mechanism/driver_inputs.hpp#L166).*

<a id="button_r1_does_two_jobs"></a>

## `button_R1_does_two_jobs`

```cpp
inline void button_R1_does_two_jobs()
```

R1 does two jobs (n17).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:168`](../../include/shulib/mechanism/driver_inputs.hpp#L168).*

<a id="button_r2_does_two_jobs"></a>

## `button_R2_does_two_jobs`

```cpp
inline void button_R2_does_two_jobs()
```

R2 does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:170`](../../include/shulib/mechanism/driver_inputs.hpp#L170).*

<a id="button_up_does_two_jobs"></a>

## `button_Up_does_two_jobs`

```cpp
inline void button_Up_does_two_jobs()
```

Up does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:172`](../../include/shulib/mechanism/driver_inputs.hpp#L172).*

<a id="button_down_does_two_jobs"></a>

## `button_Down_does_two_jobs`

```cpp
inline void button_Down_does_two_jobs()
```

Down does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:174`](../../include/shulib/mechanism/driver_inputs.hpp#L174).*

<a id="button_left_does_two_jobs"></a>

## `button_Left_does_two_jobs`

```cpp
inline void button_Left_does_two_jobs()
```

Left does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:176`](../../include/shulib/mechanism/driver_inputs.hpp#L176).*

<a id="button_right_does_two_jobs"></a>

## `button_Right_does_two_jobs`

```cpp
inline void button_Right_does_two_jobs()
```

Right does two jobs (m2).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:178`](../../include/shulib/mechanism/driver_inputs.hpp#L178).*

<a id="button_x_does_two_jobs"></a>

## `button_X_does_two_jobs`

```cpp
inline void button_X_does_two_jobs()
```

X does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:180`](../../include/shulib/mechanism/driver_inputs.hpp#L180).*

<a id="button_b_does_two_jobs"></a>

## `button_B_does_two_jobs`

```cpp
inline void button_B_does_two_jobs()
```

B does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:182`](../../include/shulib/mechanism/driver_inputs.hpp#L182).*

<a id="button_y_does_two_jobs"></a>

## `button_Y_does_two_jobs`

```cpp
inline void button_Y_does_two_jobs()
```

Y does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:184`](../../include/shulib/mechanism/driver_inputs.hpp#L184).*

<a id="button_a_does_two_jobs"></a>

## `button_A_does_two_jobs`

```cpp
inline void button_A_does_two_jobs()
```

A does two jobs.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:186`](../../include/shulib/mechanism/driver_inputs.hpp#L186).*

<a id="a_hold_needs_volts_above_0_and_at_most_12"></a>

## `a_hold_needs_volts_above_0_and_at_most_12`

```cpp
inline void a_hold_needs_volts_above_0_and_at_most_12()
```

A hold's volts outside (0, 12] (n20).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:188`](../../include/shulib/mechanism/driver_inputs.hpp#L188).*

<a id="a_preset_or_a_homing_needs_a_time_limit_above_zero"></a>

## `a_preset_or_a_homing_needs_a_time_limit_above_zero`

```cpp
inline void a_preset_or_a_homing_needs_a_time_limit_above_zero()
```

A preset or a homing control with no time limit (n34).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:190`](../../include/shulib/mechanism/driver_inputs.hpp#L190).*

<a id="two_controls_of_the_same_kind_on_one_mechanism"></a>

## `two_controls_of_the_same_kind_on_one_mechanism`

```cpp
inline void two_controls_of_the_same_kind_on_one_mechanism()
```

Two controls of one kind on one mechanism — two holds, two toggles, two homings, two presets to one target (n21). A hold beside presets is fine: the most recent input wins (Q8).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:193`](../../include/shulib/mechanism/driver_inputs.hpp#L193).*

<a id="a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed"></a>

## `a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed`

```cpp
inline void a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed()
```

A toggle on a three-position pneumatic, or on a flywheel's constant speed instead of a named one (n22).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:196`](../../include/shulib/mechanism/driver_inputs.hpp#L196).*

<a id="a_homing_control_on_a_mechanism_that_is_never_homed"></a>

## `a_homing_control_on_a_mechanism_that_is_never_homed`

```cpp
inline void a_homing_control_on_a_mechanism_that_is_never_homed()
```

A homing control on a mechanism declared neverHomed.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:198`](../../include/shulib/mechanism/driver_inputs.hpp#L198).*

<a id="a_mechanism_has_no_driver_control_mark_it_autononly_if_that_is_meant"></a>

## `a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant`

```cpp
inline void a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant()
```

A mechanism with no control and not marked autonOnly (n18).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:200`](../../include/shulib/mechanism/driver_inputs.hpp#L200).*

<a id="a_mechanism_marked_autononly_also_has_a_control"></a>

## `a_mechanism_marked_autonOnly_also_has_a_control`

```cpp
inline void a_mechanism_marked_autonOnly_also_has_a_control()
```

A mechanism marked autonOnly that also has a control.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:202`](../../include/shulib/mechanism/driver_inputs.hpp#L202).*

<a id="more_controls_than_fit"></a>

## `more_controls_than_fit`

```cpp
inline void more_controls_than_fit()
```

More than DriverControls::kCap controls.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:204`](../../include/shulib/mechanism/driver_inputs.hpp#L204).*

<a id="a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control"></a>

## `a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control`

```cpp
inline void a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control()
```

A switch- or stall-homed mechanism held or preset with no way to home it in driver control (n48): its buttons would be dead after any restart off its home — add homeAtDriverStart or homeOnPress. The declaration's `.unhomedVolts` answers for its HOLDS only (they move it unhomed at those volts); it is never a way to HOME it — a jog never zeroes a mechanism — so a PRESET on it would end NOT_HOMED for the rest of the run, and a preset needs one of the two homing controls.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:210`](../../include/shulib/mechanism/driver_inputs.hpp#L210).*

<a id="a_never_homed_mechanism_needs_unhomedvolts_for_its_buttons_to_move_it"></a>

## `a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it`

```cpp
inline void a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it()
```

A never-homed mechanism held with no `.unhomedVolts` (n49): its buttons would command 0 V.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:212`](../../include/shulib/mechanism/driver_inputs.hpp#L212).*

<a id="a_preset_on_a_mechanism_that_is_never_homed"></a>

## `a_preset_on_a_mechanism_that_is_never_homed`

```cpp
inline void a_preset_on_a_mechanism_that_is_never_homed()
```

A preset on a never-homed mechanism (n50): always NOT_HOMED — a dead button.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:214`](../../include/shulib/mechanism/driver_inputs.hpp#L214).*

<a id="whileheld"></a>

## `whileHeld`

```cpp
[[nodiscard]] constexpr Control whileHeld(const Power& m, hal::ControllerButton forward, hal::ControllerButton back, units::Voltage volts) noexcept
```

WHILE HELD on a POWER mechanism: `forward` held = +volts, `back` held = −volts, let go = stop (its declared safe state). (Re-check 2026-10-07: "hold" read as a position AND a brake mode.)

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:221`](../../include/shulib/mechanism/driver_inputs.hpp#L221).*

<a id="whileheld-2"></a>

## `whileHeld (overload 2)`

```cpp
[[nodiscard]] constexpr Control whileHeld(const Power& m, hal::ControllerButton run, units::Voltage volts) noexcept
```

WHILE HELD, ONE BUTTON, on a POWER mechanism (chunk ROB1-C2): `run` held = +volts — ONE way, the declaration's `.motors` direction — let go = stop (its declared safe state). No back button: the button beside it stays free for another control.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:229`](../../include/shulib/mechanism/driver_inputs.hpp#L229).*

<a id="whileheld-3"></a>

## `whileHeld (overload 3)`

```cpp
template <class D> consteval Control whileHeld(const D&, hal::ControllerButton, units::Voltage)
```

`whileHeld(arm, Button::R1, 6_volt)` — one button on anything but a power mechanism: refused BY NAME (an arm's or a lift's jog needs a button each way; a pneumatic or a flywheel is toggled or preset).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:237`](../../include/shulib/mechanism/driver_inputs.hpp#L237).*

<a id="whileheld-4"></a>

## `whileHeld (overload 4)`

```cpp
[[nodiscard]] constexpr Control whileHeld(const Arm& m, hal::ControllerButton up, hal::ControllerButton down, units::Voltage volts) noexcept
```

WHILE HELD on an ARM: `up` held = +volts (a jog), `down` held = −volts, let go = HOLD HERE.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:242`](../../include/shulib/mechanism/driver_inputs.hpp#L242).*

<a id="whileheld-5"></a>

## `whileHeld (overload 5)`

```cpp
[[nodiscard]] constexpr Control whileHeld(const Lift& m, hal::ControllerButton up, hal::ControllerButton down, units::Voltage volts) noexcept
```

WHILE HELD on a LIFT: as on an arm.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:249`](../../include/shulib/mechanism/driver_inputs.hpp#L249).*

<a id="whileheld-6"></a>

## `whileHeld (overload 6)`

```cpp
template <class D> consteval Control whileHeld(const D&, hal::ControllerButton, hal::ControllerButton, units::Voltage)
```

`whileHeld(hood, ...)` / `whileHeld(flywheel, ...)`: refused BY NAME (use toggle or preset).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:258`](../../include/shulib/mechanism/driver_inputs.hpp#L258).*

<a id="toggle"></a>

## `toggle`

```cpp
[[nodiscard]] constexpr Control toggle(const Pneumatic& m, hal::ControllerButton b) noexcept
```

Each SEEN press flips a TWO-position pneumatic.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:263`](../../include/shulib/mechanism/driver_inputs.hpp#L263).*

<a id="toggle-2"></a>

## `toggle (overload 2)`

```cpp
[[nodiscard]] constexpr Control toggle(SpeedGoal g, hal::ControllerButton b) noexcept
```

Each SEEN press spins a flywheel up to THE NAMED SPEED, or stops it: `toggle(flywheel["shoot"], Button::X)` (re-check 2026-10-07: "its first preset" changed silently when presets were reordered).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:269`](../../include/shulib/mechanism/driver_inputs.hpp#L269).*

<a id="toggle-3"></a>

## `toggle (overload 3)`

```cpp
template <class D> consteval Control toggle(const D&, hal::ControllerButton)
```

`toggle(flywheel, X)`: refused BY NAME — name the speed (n51).

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:276`](../../include/shulib/mechanism/driver_inputs.hpp#L276).*

<a id="toggle-4"></a>

## `toggle (overload 4)`

```cpp
template <class D> consteval Control toggle(const D&, hal::ControllerButton)
```

`toggle(intake, X)` / `toggle(arm, X)`: refused BY NAME.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:283`](../../include/shulib/mechanism/driver_inputs.hpp#L283).*

<a id="preset"></a>

## `preset`

```cpp
[[nodiscard]] constexpr Control preset(PositionGoal g, hal::ControllerButton b, units::Time bound) noexcept
```

A SEEN press sends an arm or a lift to `g` (`arm["score"]`, `arm.to(95)`, `lift.to(18_in)`) within `bound`, then it holds there.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:289`](../../include/shulib/mechanism/driver_inputs.hpp#L289).*

<a id="preset-2"></a>

## `preset (overload 2)`

```cpp
[[nodiscard]] constexpr Control preset(SpeedGoal g, hal::ControllerButton b, units::Time bound) noexcept
```

A SEEN press spins a flywheel up to `g` (`flywheel["shoot"]`, `flywheel.at(3000)`) within `bound` (its "at speed" verdict); it keeps spinning.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:296`](../../include/shulib/mechanism/driver_inputs.hpp#L296).*

<a id="preset-3"></a>

## `preset (overload 3)`

```cpp
[[nodiscard]] constexpr Control preset(PneumaticGoal g, hal::ControllerButton b) noexcept
```

A SEEN press puts a pneumatic in position `g` (`claw["open"]`) — no time limit: its move time is the robot file's.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:303`](../../include/shulib/mechanism/driver_inputs.hpp#L303).*

<a id="homeonpress"></a>

## `homeOnPress`

```cpp
[[nodiscard]] constexpr Control homeOnPress(const Arm& m, hal::ControllerButton b, units::Time bound) noexcept
```

A SEEN press homes an ARM by its declaration's method, within `bound`.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:309`](../../include/shulib/mechanism/driver_inputs.hpp#L309).*

<a id="homeonpress-2"></a>

## `homeOnPress (overload 2)`

```cpp
[[nodiscard]] constexpr Control homeOnPress(const Lift& m, hal::ControllerButton b, units::Time bound) noexcept
```

A SEEN press homes a LIFT by its declaration's method, within `bound`.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:314`](../../include/shulib/mechanism/driver_inputs.hpp#L314).*

<a id="homeatdriverstart"></a>

## `homeAtDriverStart`

```cpp
[[nodiscard]] constexpr Control homeAtDriverStart(const Arm& m, units::Time bound) noexcept
```

ONCE at driver-control entry, an ARM nothing has homed is homed, within `bound`.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:319`](../../include/shulib/mechanism/driver_inputs.hpp#L319).*

<a id="homeatdriverstart-2"></a>

## `homeAtDriverStart (overload 2)`

```cpp
[[nodiscard]] constexpr Control homeAtDriverStart(const Lift& m, units::Time bound) noexcept
```

ONCE at driver-control entry, a LIFT nothing has homed is homed, within `bound`.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:324`](../../include/shulib/mechanism/driver_inputs.hpp#L324).*

<a id="homeonpress-3"></a>

## `homeOnPress (overload 3)`

```cpp
template <class D> consteval Control homeOnPress(const D&, hal::ControllerButton, units::Time)
```

`homeOnPress(intake, ...)`: refused BY NAME — only an arm or a lift homes.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:331`](../../include/shulib/mechanism/driver_inputs.hpp#L331).*

<a id="homeatdriverstart-3"></a>

## `homeAtDriverStart (overload 3)`

```cpp
template <class D> consteval Control homeAtDriverStart(const D&, units::Time)
```

`homeAtDriverStart(intake, ...)`: refused BY NAME — only an arm or a lift homes.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:338`](../../include/shulib/mechanism/driver_inputs.hpp#L338).*

<a id="autononly"></a>

## `autonOnly`

```cpp
template <class Declared> [[nodiscard]] constexpr Control autonOnly(const Declared& m) noexcept
```

NO driver control, on purpose (Q8): `autonOnly(hood)`.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:345`](../../include/shulib/mechanism/driver_inputs.hpp#L345).*

<a id="struct-drivercontrols"></a>

## `struct DriverControls`

```cpp
struct DriverControls
```

A driver file's controls, checked (controls(...)).

*struct, declared at [`include/shulib/mechanism/driver_inputs.hpp:385`](../../include/shulib/mechanism/driver_inputs.hpp#L385).*

<a id="drivercontrols-kcap"></a>

### `DriverControls::kCap`

```cpp
static constexpr std::size_t kCap = 24
```

The most controls one driver file holds.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:387`](../../include/shulib/mechanism/driver_inputs.hpp#L387).*

<a id="drivercontrols-c"></a>

### `DriverControls::c`

```cpp
Control c[kCap]{}
```

The controls, in the order written.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:389`](../../include/shulib/mechanism/driver_inputs.hpp#L389).*

<a id="drivercontrols-count"></a>

### `DriverControls::count`

```cpp
std::uint8_t count = 0
```

How many are real.

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:391`](../../include/shulib/mechanism/driver_inputs.hpp#L391).*

<a id="drivercontrols-all"></a>

### `DriverControls::all`

```cpp
[[nodiscard]] constexpr std::span<const Control> all() const noexcept
```

The controls as a span.

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:393`](../../include/shulib/mechanism/driver_inputs.hpp#L393).*

<a id="drivercontrols-avoids"></a>

### `DriverControls::avoids`

```cpp
[[nodiscard]] constexpr bool avoids(hal::ControllerButton b) const noexcept
```

No control takes `b` — the driver loop's reserved button (its re-zero).

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:397`](../../include/shulib/mechanism/driver_inputs.hpp#L397).*

<a id="controls"></a>

## `controls`

```cpp
template <class... C> [[nodiscard]] consteval DriverControls controls(const MechanismTable& robot, C... list)
```

THE DRIVER FILE'S ONE LIST (file header), checked AT BUILD against the robot's list `robot`, each rule refused BY NAME (refused::): every control names a mechanism this robot has; no button does two jobs (the button named; a one-button hold takes its one); a hold's volts in (0, 12]; a preset's and a homing's time limit above 0 (a pneumatic preset states none); no two controls of one kind on one mechanism (a hold beside presets is fine — Q8; two presets are two kinds of one only when they aim at the same target); a toggle only on a two-position pneumatic or a flywheel's NAMED speed; a homing control only on a mechanism that homes; a switch- or stall-homed arm or lift preset only with a way to home it in driver control (homeAtDriverStart or homeOnPress), and held only with one of those or its `.unhomedVolts` (U5 — a jog moves it unhomed but never homes it); a never-homed one held only with `.unhomedVolts`, and never preset; every mechanism of the list has a control or is marked autonOnly (and then has none); at most DriverControls::kCap controls.

*free function, declared at [`include/shulib/mechanism/driver_inputs.hpp:419`](../../include/shulib/mechanism/driver_inputs.hpp#L419).*

<a id="class-driverinputs"></a>

## `class DriverInputs`

```cpp
class DriverInputs
```

A driver file's controls at run time, bound to the robot's mechanism service (file header). Lives in the program's static storage beside the service; bind() once, at boot, after the service. Non-copyable and non-movable: the loop holds it by reference, and a copy would be a second set of button states commanding the same mechanisms.

*class, declared at [`include/shulib/mechanism/driver_inputs.hpp:547`](../../include/shulib/mechanism/driver_inputs.hpp#L547).*

<a id="driverinputs-kcap"></a>

### `DriverInputs::kCap`

```cpp
static constexpr std::size_t kCap = DriverControls::kCap
```

The most controls (DriverControls::kCap).

*field, declared at [`include/shulib/mechanism/driver_inputs.hpp:550`](../../include/shulib/mechanism/driver_inputs.hpp#L550).*

<a id="driverinputs-driverinputs"></a>

### `DriverInputs::DriverInputs`

```cpp
DriverInputs() = default
```

Unbound: it commands nothing until bind().

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:553`](../../include/shulib/mechanism/driver_inputs.hpp#L553).*

<a id="driverinputs-driverinputs-2"></a>

### `DriverInputs::DriverInputs (overload 2)`

```cpp
DriverInputs(const DriverInputs&) = delete
```

Unbound: it commands nothing until bind(). Non-copyable (the class's reason).

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:555`](../../include/shulib/mechanism/driver_inputs.hpp#L555).*

<a id="driverinputs-operator-eq"></a>

### `DriverInputs::operator=`

```cpp
DriverInputs& operator=(const DriverInputs&) = delete
```

Unbound: it commands nothing until bind(). Non-copyable (the class's reason). Non-copy-assignable (the class's reason).

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:557`](../../include/shulib/mechanism/driver_inputs.hpp#L557).*

<a id="driverinputs-driverinputs-3"></a>

### `DriverInputs::DriverInputs (overload 3)`

```cpp
DriverInputs(DriverInputs&&) = delete
```

Unbound: it commands nothing until bind(). Non-copyable (the class's reason). Non-copy-assignable (the class's reason). Non-movable (the class's reason).

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:559`](../../include/shulib/mechanism/driver_inputs.hpp#L559).*

<a id="driverinputs-operator-eq-2"></a>

### `DriverInputs::operator= (overload 2)`

```cpp
DriverInputs& operator=(DriverInputs&&) = delete
```

Unbound: it commands nothing until bind(). Non-copyable (the class's reason). Non-copy-assignable (the class's reason). Non-movable (the class's reason). Non-move-assignable (the class's reason).

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:561`](../../include/shulib/mechanism/driver_inputs.hpp#L561).*

<a id="driverinputs-destructor-driverinputs"></a>

### `DriverInputs::~DriverInputs`

```cpp
~DriverInputs() = default
```

Unbound: it commands nothing until bind(). Non-copyable (the class's reason). Non-copy-assignable (the class's reason). Non-movable (the class's reason). Non-move-assignable (the class's reason). Nothing to release: the service owns every claim.

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:563`](../../include/shulib/mechanism/driver_inputs.hpp#L563).*

<a id="driverinputs-bind"></a>

### `DriverInputs::bind`

```cpp
void bind(MechanismService& svc, const DriverControls& controls)
```

BIND, once, at boot, after `svc` is bound: each control resolved to its slot by the declaration's NAME. `controls` is BORROWED (static storage: a driver file's `inline constexpr`). A control on a mechanism that is ABSENT this boot still binds — it commands a slot that does nothing (the drive runs). PRECONDITIONS (the controls were checked against ONE robot's list at build; these check them against the robot the SERVICE is bound to, so a driver file built for another robot never commands a stranger): not bound yet; `svc` bound; every control but an autonOnly names a mechanism the service has, of the control's kind and units; a toggle's pneumatic has two positions; a named preset (or a flywheel toggle's named speed) is this mechanism's, at the same value; a constant target is inside its travel limits or free speed; a pneumatic preset's position is this mechanism's, by name; a homing control's mechanism homes.

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:575`](../../include/shulib/mechanism/driver_inputs.hpp#L575).*

<a id="driverinputs-avoids"></a>

### `DriverInputs::avoids`

```cpp
[[nodiscard]] bool avoids(hal::ControllerButton b) const noexcept
```

No control takes `b` — the loop's re-zero button. True when unbound.

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:604`](../../include/shulib/mechanism/driver_inputs.hpp#L604).*

<a id="driverinputs-bound"></a>

### `DriverInputs::bound`

```cpp
[[nodiscard]] bool bound() const noexcept
```

True once bind() has run.

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:608`](../../include/shulib/mechanism/driver_inputs.hpp#L608).*

<a id="driverinputs-boundto"></a>

### `DriverInputs::boundTo`

```cpp
[[nodiscard]] bool boundTo(const MechanismService& svc) const noexcept
```

True when bind() bound it to `svc` — the loop's check that its inputs command the service it ticks.

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:611`](../../include/shulib/mechanism/driver_inputs.hpp#L611).*

<a id="driverinputs-enter"></a>

### `DriverInputs::enter`

```cpp
void enter()
```

DRIVER-CONTROL ENTRY (after the service's enterDriverControl()): every hold released; every toggle's, preset's and homeOnPress's button counted as HELD (a press must be SEEN — one already down on the entry tick does nothing until released and pressed again; a hold's button is a level, and down at entry it drives at once — file header); then each homeAtDriverStart homes its mechanism when nothing has homed it (skipped when homed — by the auton, at boot, or an earlier entry), and while that home runs the mechanism's own controls wait (file header). PRECONDITION: bound.

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:620`](../../include/shulib/mechanism/driver_inputs.hpp#L620).*

<a id="driverinputs-tick"></a>

### `DriverInputs::tick`

```cpp
void tick(const hal::IController& c, bool connected)
```

ONE DRIVER TICK: read every control's buttons (only while `connected`) and command the service (file header) — the service ticks right after (teleop_loop.hpp). PRECONDITION: bound.

*function, declared at [`include/shulib/mechanism/driver_inputs.hpp:641`](../../include/shulib/mechanism/driver_inputs.hpp#L641).*

<a id="drivercontrols"></a>

## `DriverControls`

```cpp
using mechanism::DriverControls
```

A driver file's checked list of controls.

*type alias, declared at [`include/shulib/mechanism/driver_inputs.hpp:825`](../../include/shulib/mechanism/driver_inputs.hpp#L825).*

<a id="controls-2"></a>

## `controls (overload 2)`

```cpp
using mechanism::controls
```

The list itself: `controls(kRobotXMechanisms, ...)`.

*type alias, declared at [`include/shulib/mechanism/driver_inputs.hpp:827`](../../include/shulib/mechanism/driver_inputs.hpp#L827).*

<a id="whileheld-7"></a>

## `whileHeld (overload 7)`

```cpp
using mechanism::whileHeld
```

Hold-to-run: two buttons ± volts while held, or one button + volts on a power mechanism (oneWay).

*type alias, declared at [`include/shulib/mechanism/driver_inputs.hpp:829`](../../include/shulib/mechanism/driver_inputs.hpp#L829).*

<a id="toggle-5"></a>

## `toggle (overload 5)`

```cpp
using mechanism::toggle
```

One button flips a two-position pneumatic, or a flywheel's named speed on / off.

*type alias, declared at [`include/shulib/mechanism/driver_inputs.hpp:831`](../../include/shulib/mechanism/driver_inputs.hpp#L831).*

<a id="preset-4"></a>

## `preset (overload 4)`

```cpp
using mechanism::preset
```

One button: a preset, a constant, a speed or a pneumatic position.

*type alias, declared at [`include/shulib/mechanism/driver_inputs.hpp:833`](../../include/shulib/mechanism/driver_inputs.hpp#L833).*

<a id="homeonpress-4"></a>

## `homeOnPress (overload 4)`

```cpp
using mechanism::homeOnPress
```

One button homes an arm or a lift.

*type alias, declared at [`include/shulib/mechanism/driver_inputs.hpp:835`](../../include/shulib/mechanism/driver_inputs.hpp#L835).*

<a id="homeatdriverstart-4"></a>

## `homeAtDriverStart (overload 4)`

```cpp
using mechanism::homeAtDriverStart
```

An arm or a lift nothing homed is homed once at driver-control start.

*type alias, declared at [`include/shulib/mechanism/driver_inputs.hpp:837`](../../include/shulib/mechanism/driver_inputs.hpp#L837).*

<a id="autononly-2"></a>

## `autonOnly (overload 2)`

```cpp
using mechanism::autonOnly
```

No driver control, on purpose.

*type alias, declared at [`include/shulib/mechanism/driver_inputs.hpp:839`](../../include/shulib/mechanism/driver_inputs.hpp#L839).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 78 lines, click to expand</summary>

```text

 mechanism/driver_inputs.hpp — THE DRIVER FILE'S MECHANISM CONTROLS: hold-to-run, toggle, preset and
 the homing controls, each naming its mechanism by its DECLARATION, checked at build against the
 robot's list, and run THROUGH the mechanism service (the mechanism core, chunk MECH-A4, 2026-10-22).

 ── What a driver file writes ────────────────────────────────────────────────────────
 ONE list, checked AT BUILD (consteval controls(...): every rule below is made there, so the file
 carries no static_assert of its own):

     inline constexpr DriverControls kControls = controls(kRobotXMechanisms,
         whileHeld(arm, Button::R1, Button::R2, 6_volt),  // up / down while held; let go: it holds there
         preset(arm["score"], Button::Right, 1.5_s),      // the most recent button wins
         whileHeld(lift, Button::L1, Button::L2, 12_volt),
         homeAtDriverStart(lift, 1.5_s),                  // after a restart it may not be on its switch
         toggle(flywheel["shoot"], Button::X),            // on at "shoot" / off
         whileHeld(intake, Button::A, Button::B, 12_volt),
         whileHeld(sideRoller, Button::Up, 8_volt),      // ONE button: runs one way while held, stops on release
         autonOnly(hood));                                // the hood is the autons' alone

 ── What each control does (DriverInputs::tick, once per driver-control tick) ──────────
   * whileHeld on a POWER mechanism: held → drive(±V) every tick; released → stop (its safe state).
     With ONE button (`whileHeld(sideRoller, Button::Up, 8_volt)`, chunk ROB1-C2): held → drive(+V),
     released → stop — it runs ONE way only (which way is its declaration's `.motors` direction), and
     takes no second button. A POWER mechanism only (an arm's or a lift's jog needs both ways).
     On an ARM or a LIFT: held → jog(±V) (soft-stopped at the limits once homed; at most
     `.unhomedVolts` before); released → HOLD HERE (gravity included, clamped to the range). Both
     buttons held = neither (T3's ruling R2): the release rule.
   * toggle: a SEEN press flips a two-position pneumatic, or spins a flywheel up to the NAMED speed
     (`toggle(flywheel["shoot"], X)`) / stops it. A flywheel toggle's step is bounded by
     kToggleAtSpeedWindowSeconds — only its logged "at speed" verdict: the wheel keeps spinning.
   * preset: a SEEN press sends the mechanism to a preset or a constant (`arm["score"]`,
     `lift.to(18_in)`, `flywheel["shoot"]`, `claw["open"]`) within its time limit; then it holds or
     keeps spinning. A pneumatic position states no time limit (its move time is the robot file's).
   * homeOnPress: a SEEN press homes it by its declaration's method, bounded. homeAtDriverStart:
     ONCE at driver-control entry, when nothing has homed it (a restart can leave a switch- or
     stall-homed mechanism off its home: its buttons would be dead — the brief's U5). WHILE THAT
     HOME RUNS, the mechanism's own holds, presets and homing press WAIT (the exception below).
   * autonOnly: no driver control, on purpose (Q8) — the build's "every mechanism has a control"
     rule then passes it.

 ── THE MOST RECENT INPUT WINS (the team lead's Q8 (a)) ─────────────────────────────────
 Arbitration is by the service slot's COMMAND NUMBER (Slot::commandSeq(), bumped by every command
 and by a stall that ends a behaviour after its step). A hold-to-run that sees a number it did not
 write has been SUPERSEDED — by a preset, a toggle, another control, an auton leftover, a stall —
 and commands NOTHING until its buttons are seen released while connected (T3's "an outside stop
 is never undone by a finger that never left the button", generalized).

 A PRESS MUST BE SEEN — for a toggle, a preset and a homeOnPress: at entry its button counts as
 held (one already down on the entry tick does nothing until it is released and pressed again),
 and so does its button while the controller is disconnected (a drop is never a press, and a
 reconnect with the button still down is not one either). A HOLD is a level, not a press (T3's
 rule for a hold binding): its button already down on the entry tick drives at once, and a
 disconnect releases it that tick. So a disconnect is a release for the holds, never a press for
 the rest. All of it rests on one unmeasured belief, T3's ToggleBinding's too: the controller's
 connected flag and its button levels come back TOGETHER (A4 register HA-147, invented) — the
 first connected tick's level is taken as live, so a stale "up" read on it while the driver still
 holds the button would make the next live "down" a press nobody made.

 THE ONE EXCEPTION — the driver-start home (U5). While a homeAtDriverStart home is running, that
 mechanism's OWN controls wait: a hold held during it is SUPERSEDED (nothing until its buttons are
 seen released after the home ends), and a preset's or a homeOnPress's press is consumed and
 dropped. Without it, a driver touching the lift in the home's first 1.5 s would CANCEL the home
 ("replaced by a newer command") and leave the lift unhomed — every preset NOT_HOMED and every
 hold at `.unhomedVolts` (0: nothing) for the rest of the match, the dead buttons U5 exists to
 prevent. Waiting costs at most the home's own time limit. Other mechanisms' controls never wait.

 ── Where it runs ───────────────────────────────────────────────────────────────────
 teleop::runDriverControl's service overload (teleop/teleop_loop.hpp) calls enter() at loop entry
 (after the service's enterDriverControl()) and tick() each tick after the T3 bindings' place and
 BEFORE the service's tickFromDriverLoop() — so a press read at tick k is commanded AND ticked in
 tick k — and before the loop's perTick.

 ── Static storage ──────────────────────────────────────────────────────────────────
 A program keeps its DriverInputs in static storage beside its service, like T3's bindings: PROS
 deletes the driver task on a mode change (HA-161) and a deleted task runs no destructor. bind()
 once, at boot, AFTER the service is bound.

 PROS-free, allocation-free. Nothing here has run on a robot.
```

</details>
