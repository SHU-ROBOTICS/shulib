<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/wiring.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `wiring.hpp`

mechanism/wiring.hpp — THE BINDING (the mechanism core, chunk MECH-A1, 2026-10-08): which port each ROLE is plugged into, a block of the robot file SEPARATE from the declarations, compiled in (P2; the team lead's Q1 (a): the binding lives only in the robot…

This header declares **9** types (62 members), **81** free functions, and **8** type aliass.

Extracted from [`include/shulib/mechanism/wiring.hpp`](../../include/shulib/mechanism/wiring.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class Line`](#enum-class-line)
  - [`None`](#line-none)
  - [`A`](#line-a)
  - [`B`](#line-b)
  - [`C`](#line-c)
  - [`D`](#line-d)
  - [`E`](#line-e)
  - [`F`](#line-f)
  - [`G`](#line-g)
  - [`H`](#line-h)
- [`lineLetter`](#lineletter) — *free function*
- [`struct MechanismWiring`](#struct-mechanismwiring)
  - [`mechanism`](#mechanismwiring-mechanism)
  - [`motors`](#mechanismwiring-motors)
  - [`lines`](#mechanismwiring-lines)
  - [`angleSensor`](#mechanismwiring-anglesensor)
  - [`homeSwitch`](#mechanismwiring-homeswitch)
  - [`pieceSensorPort`](#mechanismwiring-piecesensorport)
  - [`pieceSwitch`](#mechanismwiring-pieceswitch)
  - [`file`](#mechanismwiring-file)
  - [`line`](#mechanismwiring-line)
- [`struct MechanismTable`](#struct-mechanismtable)
  - [`specs`](#mechanismtable-specs)
  - [`wiring`](#mechanismtable-wiring)
  - [`count`](#mechanismtable-count)
  - [`declared`](#mechanismtable-declared)
  - [`indexOf`](#mechanismtable-indexof)
  - [`specSpan`](#mechanismtable-specspan)
  - [`wiringSpan`](#mechanismtable-wiringspan)
- [`enum class EntryProblem`](#enum-class-entryproblem)
  - [`None`](#entryproblem-none)
  - [`NameDiffers`](#entryproblem-namediffers)
  - [`MotorRoleUnbound`](#entryproblem-motorroleunbound)
  - [`SmartPortPast21`](#entryproblem-smartportpast21)
  - [`MotorPortExtra`](#entryproblem-motorportextra)
  - [`LineRoleUnbound`](#entryproblem-lineroleunbound)
  - [`LinePastH`](#entryproblem-linepasth)
  - [`LineExtra`](#entryproblem-lineextra)
  - [`LevelsVsLines`](#entryproblem-levelsvslines)
  - [`AngleSensorUnplugged`](#entryproblem-anglesensorunplugged)
  - [`AngleSensorExtra`](#entryproblem-anglesensorextra)
  - [`HomeSwitchUnplugged`](#entryproblem-homeswitchunplugged)
  - [`HomeSwitchExtra`](#entryproblem-homeswitchextra)
  - [`PieceSensorUnplugged`](#entryproblem-piecesensorunplugged)
  - [`PieceSensorExtra`](#entryproblem-piecesensorextra)
  - [`PieceSwitchUnplugged`](#entryproblem-pieceswitchunplugged)
  - [`PieceSwitchExtra`](#entryproblem-pieceswitchextra)
- [`entryProblem`](#entryproblem) — *free function*
- [`entryProblemText`](#entryproblemtext) — *free function*
- [`wiringProblem`](#wiringproblem) — *free function*
- [`toMechanismConfig`](#tomechanismconfig) — *free function*
- [`struct MotorPorts`](#struct-motorports)
  - [`port`](#motorports-port)
  - [`count`](#motorports-count)
  - [`bad`](#motorports-bad)
  - [`file`](#motorports-file)
  - [`line`](#motorports-line)
- [`struct LinePorts`](#struct-lineports)
  - [`line`](#lineports-line)
  - [`count`](#lineports-count)
  - [`bad`](#lineports-bad)
  - [`file`](#lineports-file)
  - [`lineNo`](#lineports-lineno)
- [`a_motor_port_is_a_number_like_6_a_letter_in_quotes_is_a_three_wire_port`](#a_motor_port_is_a_number_like_6_a_letter_in_quotes_is_a_three_wire_port) — *free function*
- [`a_three_wire_port_is_a_letter_in_quotes_like_A_a_number_is_a_smart_port`](#a_three_wire_port_is_a_letter_in_quotes_like_a_a_number_is_a_smart_port) — *free function*
- [`an_angle_sensor_is_a_smart_sensor_give_it_a_port_number`](#an_angle_sensor_is_a_smart_sensor_give_it_a_port_number) — *free function*
- [`a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors`](#a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors) — *free function*
- [`a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines`](#a_three_wire_port_is_not_a_letter_from_a_to_h_or_more_than_two_lines) — *free function*
- [`the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has`](#the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has) — *free function*
- [`the_mechanism_has_a_sensor_the_wiring_does_not_plug_in`](#the_mechanism_has_a_sensor_the_wiring_does_not_plug_in) — *free function*
- [`the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have`](#the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have) — *free function*
- [`a_piece_sensor_is_a_smart_sensor_give_it_a_port_number`](#a_piece_sensor_is_a_smart_sensor_give_it_a_port_number) — *free function*
- [`a_limit_switch_is_a_three_wire_device_give_it_a_letter`](#a_limit_switch_is_a_three_wire_device_give_it_a_letter) — *free function*
- [`a_smart_port_is_used_twice`](#a_smart_port_is_used_twice) — *free function*
- [`a_three_wire_port_is_used_twice`](#a_three_wire_port_is_used_twice) — *free function*
- [`the_wiring_plugs_in_the_same_sensor_twice`](#the_wiring_plugs_in_the_same_sensor_twice) — *free function*
- [`the_wiring_names_another_mechanism_than_its_declaration`](#the_wiring_names_another_mechanism_than_its_declaration) — *free function*
- [`the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has`](#the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has) — *free function*
- [`more_mechanisms_than_a_robot_can_have`](#more_mechanisms_than_a_robot_can_have) — *free function*
- [`the_same_mechanism_is_wired_twice`](#the_same_mechanism_is_wired_twice) — *free function*
- [`two_mechanisms_share_a_name`](#two_mechanisms_share_a_name) — *free function*
- [`a_position_states_a_different_number_of_levels_than_the_wiring_has_lines`](#a_position_states_a_different_number_of_levels_than_the_wiring_has_lines) — *free function*
- [`a_pneumatic_is_wired_with_lines_like_lines_A_not_motors`](#a_pneumatic_is_wired_with_lines_like_lines_a_not_motors) — *free function*
- [`a_motor_mechanism_is_wired_with_motors_like_motors_6_not_lines`](#a_motor_mechanism_is_wired_with_motors_like_motors_6_not_lines) — *free function*
- [`smart_port_1_is_used_twice`](#smart_port_1_is_used_twice) — *free function*
- [`smart_port_2_is_used_twice`](#smart_port_2_is_used_twice) — *free function*
- [`smart_port_3_is_used_twice`](#smart_port_3_is_used_twice) — *free function*
- [`smart_port_4_is_used_twice`](#smart_port_4_is_used_twice) — *free function*
- [`smart_port_5_is_used_twice`](#smart_port_5_is_used_twice) — *free function*
- [`smart_port_6_is_used_twice`](#smart_port_6_is_used_twice) — *free function*
- [`smart_port_7_is_used_twice`](#smart_port_7_is_used_twice) — *free function*
- [`smart_port_8_is_used_twice`](#smart_port_8_is_used_twice) — *free function*
- [`smart_port_9_is_used_twice`](#smart_port_9_is_used_twice) — *free function*
- [`smart_port_10_is_used_twice`](#smart_port_10_is_used_twice) — *free function*
- [`smart_port_11_is_used_twice`](#smart_port_11_is_used_twice) — *free function*
- [`smart_port_12_is_used_twice`](#smart_port_12_is_used_twice) — *free function*
- [`smart_port_13_is_used_twice`](#smart_port_13_is_used_twice) — *free function*
- [`smart_port_14_is_used_twice`](#smart_port_14_is_used_twice) — *free function*
- [`smart_port_15_is_used_twice`](#smart_port_15_is_used_twice) — *free function*
- [`smart_port_16_is_used_twice`](#smart_port_16_is_used_twice) — *free function*
- [`smart_port_17_is_used_twice`](#smart_port_17_is_used_twice) — *free function*
- [`smart_port_18_is_used_twice`](#smart_port_18_is_used_twice) — *free function*
- [`smart_port_19_is_used_twice`](#smart_port_19_is_used_twice) — *free function*
- [`smart_port_20_is_used_twice`](#smart_port_20_is_used_twice) — *free function*
- [`smart_port_21_is_used_twice`](#smart_port_21_is_used_twice) — *free function*
- [`three_wire_port_A_is_used_twice`](#three_wire_port_a_is_used_twice) — *free function*
- [`three_wire_port_B_is_used_twice`](#three_wire_port_b_is_used_twice) — *free function*
- [`three_wire_port_C_is_used_twice`](#three_wire_port_c_is_used_twice) — *free function*
- [`three_wire_port_D_is_used_twice`](#three_wire_port_d_is_used_twice) — *free function*
- [`three_wire_port_E_is_used_twice`](#three_wire_port_e_is_used_twice) — *free function*
- [`three_wire_port_F_is_used_twice`](#three_wire_port_f_is_used_twice) — *free function*
- [`three_wire_port_G_is_used_twice`](#three_wire_port_g_is_used_twice) — *free function*
- [`three_wire_port_H_is_used_twice`](#three_wire_port_h_is_used_twice) — *free function*
- [`motors`](#motors) — *free function*
- [`motors (overload 2)`](#motors-2) — *free function*
- [`motors (overload 3)`](#motors-3) — *free function*
- [`motors (overload 4)`](#motors-4) — *free function*
- [`motors (overload 5)`](#motors-5) — *free function*
- [`motors (overload 6)`](#motors-6) — *free function*
- [`lines`](#lines) — *free function*
- [`lines (overload 2)`](#lines-2) — *free function*
- [`lines (overload 3)`](#lines-3) — *free function*
- [`lines (overload 4)`](#lines-4) — *free function*
- [`struct RolePort`](#struct-roleport)
  - [`port`](#roleport-port)
  - [`line`](#roleport-line)
  - [`letter`](#roleport-letter)
  - [`bad`](#roleport-bad)
  - [`enum class RolePort::Role`](#enum-class-roleport-role)
    - [`Angle`](#roleport-role-angle)
    - [`Piece`](#roleport-role-piece)
    - [`HomeSwitch`](#roleport-role-homeswitch)
- [`angleSensor`](#anglesensor) — *free function*
- [`pieceSensor`](#piecesensor) — *free function*
- [`homeSwitch`](#homeswitch) — *free function*
- [`struct Wired`](#struct-wired)
  - [`spec`](#wired-spec)
  - [`wiring`](#wired-wiring)
  - [`declared`](#wired-declared)
- [`wire`](#wire) — *free function*
- [`wire (overload 2)`](#wire-2) — *free function*
- [`wire (overload 3)`](#wire-3) — *free function*
- [`wire (overload 4)`](#wire-4) — *free function*
- [`wire (overload 5)`](#wire-5) — *free function*
- [`wire (overload 6)`](#wire-6) — *free function*
- [`wire (overload 7)`](#wire-7) — *free function*
- [`mechanisms`](#mechanisms) — *free function*
- [`mechanisms (overload 2)`](#mechanisms-2) — *free function*
- [`clearOfDrive`](#clearofdrive) — *free function*
- [`declaredMotorPorts`](#declaredmotorports) — *free function*
- [`roleLine`](#roleline) — *free function*
- [`startingValuesLine`](#startingvaluesline) — *free function*
- [`MechanismTable`](#mechanismtable) — *type alias*
- [`mechanisms (overload 3)`](#mechanisms-3) — *type alias*
- [`wire (overload 8)`](#wire-8) — *type alias*
- [`motors (overload 7)`](#motors-7) — *type alias*
- [`lines (overload 5)`](#lines-5) — *type alias*
- [`angleSensor (overload 2)`](#anglesensor-2) — *type alias*
- [`pieceSensor (overload 2)`](#piecesensor-2) — *type alias*
- [`homeSwitch (overload 2)`](#homeswitch-2) — *type alias*

<a id="enum-class-line"></a>

## `enum class Line`

```cpp
enum class Line : std::uint8_t
```

A brain 3-wire port, A..H. None = unbound. Its own type, so a smart port number cannot be written where a line belongs (header). The value is the config's line number (A = 1).

*enum class, declared at [`include/shulib/mechanism/wiring.hpp:71`](../../include/shulib/mechanism/wiring.hpp#L71).*

<a id="line-none"></a>

### `Line::None`

```cpp
None = 0
```

unbound

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:72`](../../include/shulib/mechanism/wiring.hpp#L72).*

<a id="line-a"></a>

### `Line::A`

```cpp
A = 1
```

3-wire port A

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:73`](../../include/shulib/mechanism/wiring.hpp#L73).*

<a id="line-b"></a>

### `Line::B`

```cpp
B
```

3-wire port B

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:74`](../../include/shulib/mechanism/wiring.hpp#L74).*

<a id="line-c"></a>

### `Line::C`

```cpp
C
```

3-wire port C

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:75`](../../include/shulib/mechanism/wiring.hpp#L75).*

<a id="line-d"></a>

### `Line::D`

```cpp
D
```

3-wire port D

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:76`](../../include/shulib/mechanism/wiring.hpp#L76).*

<a id="line-e"></a>

### `Line::E`

```cpp
E
```

3-wire port E

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:77`](../../include/shulib/mechanism/wiring.hpp#L77).*

<a id="line-f"></a>

### `Line::F`

```cpp
F
```

3-wire port F

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:78`](../../include/shulib/mechanism/wiring.hpp#L78).*

<a id="line-g"></a>

### `Line::G`

```cpp
G
```

3-wire port G

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:79`](../../include/shulib/mechanism/wiring.hpp#L79).*

<a id="line-h"></a>

### `Line::H`

```cpp
H
```

3-wire port H

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:80`](../../include/shulib/mechanism/wiring.hpp#L80).*

<a id="lineletter"></a>

## `lineLetter`

```cpp
[[nodiscard]] constexpr char lineLetter(Line l) noexcept
```

A line as its letter ('A'..'H'), '-' for None.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:84`](../../include/shulib/mechanism/wiring.hpp#L84).*

<a id="struct-mechanismwiring"></a>

## `struct MechanismWiring`

```cpp
struct MechanismWiring
```

ONE mechanism's wiring: its name (checked against the spec's) and a port or line per role.

*struct, declared at [`include/shulib/mechanism/wiring.hpp:89`](../../include/shulib/mechanism/wiring.hpp#L89).*

<a id="mechanismwiring-mechanism"></a>

### `MechanismWiring::mechanism`

```cpp
char mechanism[kNameBytes]{}
```

The spec's name, in the specs' order (checked).

*field, declared at [`include/shulib/mechanism/wiring.hpp:91`](../../include/shulib/mechanism/wiring.hpp#L91).*

<a id="mechanismwiring-motors"></a>

### `MechanismWiring::motors`

```cpp
std::uint8_t motors[kMaxMotorRoles]{}
```

Smart ports 1..21, one per motor role, in the declaration's `.motors` order (physical; the direction is the declaration's).

*field, declared at [`include/shulib/mechanism/wiring.hpp:94`](../../include/shulib/mechanism/wiring.hpp#L94).*

<a id="mechanismwiring-lines"></a>

### `MechanismWiring::lines`

```cpp
Line lines[kMaxLineRoles]{}
```

3-wire lines, one per line role, in the positions' level order.

*field, declared at [`include/shulib/mechanism/wiring.hpp:96`](../../include/shulib/mechanism/wiring.hpp#L96).*

<a id="mechanismwiring-anglesensor"></a>

### `MechanismWiring::angleSensor`

```cpp
std::uint8_t angleSensor = 0
```

The angle sensor's smart port (a Rotation role); 0 = none.

*field, declared at [`include/shulib/mechanism/wiring.hpp:98`](../../include/shulib/mechanism/wiring.hpp#L98).*

<a id="mechanismwiring-homeswitch"></a>

### `MechanismWiring::homeSwitch`

```cpp
Line homeSwitch = Line::None
```

The home switch's line; None = none.

*field, declared at [`include/shulib/mechanism/wiring.hpp:100`](../../include/shulib/mechanism/wiring.hpp#L100).*

<a id="mechanismwiring-piecesensorport"></a>

### `MechanismWiring::pieceSensorPort`

```cpp
std::uint8_t pieceSensorPort = 0
```

A distance or optical piece sensor's smart port; 0 = none.

*field, declared at [`include/shulib/mechanism/wiring.hpp:102`](../../include/shulib/mechanism/wiring.hpp#L102).*

<a id="mechanismwiring-pieceswitch"></a>

### `MechanismWiring::pieceSwitch`

```cpp
Line pieceSwitch = Line::None
```

A limit-switch piece sensor's line; None = none.

*field, declared at [`include/shulib/mechanism/wiring.hpp:104`](../../include/shulib/mechanism/wiring.hpp#L104).*

<a id="mechanismwiring-file"></a>

### `MechanismWiring::file`

```cpp
const char* file = nullptr
```

Where this mechanism's `wire(...)` line is (captured where motors(...) / lines(...) is written): the direction check's "a cable of this mechanism is on another's port: check the wire(...) line HERE".

*field, declared at [`include/shulib/mechanism/wiring.hpp:108`](../../include/shulib/mechanism/wiring.hpp#L108).*

<a id="mechanismwiring-line"></a>

### `MechanismWiring::line`

```cpp
std::uint32_t line = 0
```

That line.

*field, declared at [`include/shulib/mechanism/wiring.hpp:110`](../../include/shulib/mechanism/wiring.hpp#L110).*

<a id="struct-mechanismtable"></a>

## `struct MechanismTable`

```cpp
struct MechanismTable
```

ONE ROBOT's mechanisms: the declarations as data and their wiring, in the same order — the ONE team-facing type for a robot's list (N11; the `src` build gate's word too).

*struct, declared at [`include/shulib/mechanism/wiring.hpp:115`](../../include/shulib/mechanism/wiring.hpp#L115).*

<a id="mechanismtable-specs"></a>

### `MechanismTable::specs`

```cpp
MechanismSpec specs[kMaxMechanisms]{}
```

The mechanisms as data, in boot order.

*field, declared at [`include/shulib/mechanism/wiring.hpp:117`](../../include/shulib/mechanism/wiring.hpp#L117).*

<a id="mechanismtable-wiring"></a>

### `MechanismTable::wiring`

```cpp
MechanismWiring wiring[kMaxMechanisms]{}
```

Their wiring, in the same order.

*field, declared at [`include/shulib/mechanism/wiring.hpp:119`](../../include/shulib/mechanism/wiring.hpp#L119).*

<a id="mechanismtable-count"></a>

### `MechanismTable::count`

```cpp
std::uint8_t count = 0
```

How many are real.

*field, declared at [`include/shulib/mechanism/wiring.hpp:121`](../../include/shulib/mechanism/wiring.hpp#L121).*

<a id="mechanismtable-declared"></a>

### `MechanismTable::declared`

```cpp
const void* declared[kMaxMechanisms]{}
```

The team's declarations (kinds.hpp), in the same order — what a driver control's build check compares (by address); nullptr for a hand-built table. The run-time lookup is by NAME.

*field, declared at [`include/shulib/mechanism/wiring.hpp:124`](../../include/shulib/mechanism/wiring.hpp#L124).*

<a id="mechanismtable-indexof"></a>

### `MechanismTable::indexOf`

```cpp
[[nodiscard]] constexpr int indexOf(const void* d) const noexcept
```

The index of declaration `d` in this list, or -1 when this robot does not have it. nullptr is no declaration, so it is never found — even beside hand-built entries, whose `declared` is nullptr (MECH-A1's review: two hand-built entries were taken for "the same mechanism").

*function, declared at [`include/shulib/mechanism/wiring.hpp:128`](../../include/shulib/mechanism/wiring.hpp#L128).*

<a id="mechanismtable-specspan"></a>

### `MechanismTable::specSpan`

```cpp
[[nodiscard]] constexpr std::span<const MechanismSpec> specSpan() const noexcept
```

The specs as a span (bounded by the capacity).

*function, declared at [`include/shulib/mechanism/wiring.hpp:140`](../../include/shulib/mechanism/wiring.hpp#L140).*

<a id="mechanismtable-wiringspan"></a>

### `MechanismTable::wiringSpan`

```cpp
[[nodiscard]] constexpr std::span<const MechanismWiring> wiringSpan() const noexcept
```

The wiring as a span (bounded by the capacity).

*function, declared at [`include/shulib/mechanism/wiring.hpp:144`](../../include/shulib/mechanism/wiring.hpp#L144).*

<a id="enum-class-entryproblem"></a>

## `enum class EntryProblem`

```cpp
enum class EntryProblem : std::uint8_t
```

What can be wrong with ONE mechanism's wiring beside its own spec, before any other mechanism or the drive is looked at (header) — the per-entry rules wire() enforces as it builds an entry, as data, in the order entryProblem() checks them. mechanisms() holds EVERY entry to them, a hand-built one included (it never passed through wire()), and names each (refused::); wiringProblem() checks a tool's table with the same function and words each (entryProblemText). One rule set, two readers (MECH-A1's review: a hand-built entry with an unbound motor, a sensor its spec lacks or another mechanism's name used to compile).

*enum class, declared at [`include/shulib/mechanism/wiring.hpp:156`](../../include/shulib/mechanism/wiring.hpp#L156).*

<a id="entryproblem-none"></a>

### `EntryProblem::None`

```cpp
None = 0
```

nothing is wrong with the entry on its own

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:157`](../../include/shulib/mechanism/wiring.hpp#L157).*

<a id="entryproblem-namediffers"></a>

### `EntryProblem::NameDiffers`

```cpp
NameDiffers
```

the wiring names another mechanism than its spec

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:158`](../../include/shulib/mechanism/wiring.hpp#L158).*

<a id="entryproblem-motorroleunbound"></a>

### `EntryProblem::MotorRoleUnbound`

```cpp
MotorRoleUnbound
```

a motor role the mechanism has is on port 0

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:159`](../../include/shulib/mechanism/wiring.hpp#L159).*

<a id="entryproblem-smartportpast21"></a>

### `EntryProblem::SmartPortPast21`

```cpp
SmartPortPast21
```

a bound smart port (a motor's or a sensor's) past 21

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:160`](../../include/shulib/mechanism/wiring.hpp#L160).*

<a id="entryproblem-motorportextra"></a>

### `EntryProblem::MotorPortExtra`

```cpp
MotorPortExtra
```

a motor port on a role past the mechanism's motor count

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:161`](../../include/shulib/mechanism/wiring.hpp#L161).*

<a id="entryproblem-lineroleunbound"></a>

### `EntryProblem::LineRoleUnbound`

```cpp
LineRoleUnbound
```

a line role the mechanism has is unbound

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:162`](../../include/shulib/mechanism/wiring.hpp#L162).*

<a id="entryproblem-linepasth"></a>

### `EntryProblem::LinePastH`

```cpp
LinePastH
```

a bound 3-wire line past H

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:163`](../../include/shulib/mechanism/wiring.hpp#L163).*

<a id="entryproblem-lineextra"></a>

### `EntryProblem::LineExtra`

```cpp
LineExtra
```

a 3-wire line on a role past the mechanism's line count

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:164`](../../include/shulib/mechanism/wiring.hpp#L164).*

<a id="entryproblem-levelsvslines"></a>

### `EntryProblem::LevelsVsLines`

```cpp
LevelsVsLines
```

a pneumatic position states a different number of levels than the lines (U2)

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:165`](../../include/shulib/mechanism/wiring.hpp#L165).*

<a id="entryproblem-anglesensorunplugged"></a>

### `EntryProblem::AngleSensorUnplugged`

```cpp
AngleSensorUnplugged
```

the angle sensor the mechanism has is unbound

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:166`](../../include/shulib/mechanism/wiring.hpp#L166).*

<a id="entryproblem-anglesensorextra"></a>

### `EntryProblem::AngleSensorExtra`

```cpp
AngleSensorExtra
```

an angle sensor port on a mechanism with none

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:167`](../../include/shulib/mechanism/wiring.hpp#L167).*

<a id="entryproblem-homeswitchunplugged"></a>

### `EntryProblem::HomeSwitchUnplugged`

```cpp
HomeSwitchUnplugged
```

the home switch the mechanism has is unbound

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:168`](../../include/shulib/mechanism/wiring.hpp#L168).*

<a id="entryproblem-homeswitchextra"></a>

### `EntryProblem::HomeSwitchExtra`

```cpp
HomeSwitchExtra
```

a home switch line on a mechanism with none

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:169`](../../include/shulib/mechanism/wiring.hpp#L169).*

<a id="entryproblem-piecesensorunplugged"></a>

### `EntryProblem::PieceSensorUnplugged`

```cpp
PieceSensorUnplugged
```

the distance or optical piece sensor the mechanism has is unbound

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:170`](../../include/shulib/mechanism/wiring.hpp#L170).*

<a id="entryproblem-piecesensorextra"></a>

### `EntryProblem::PieceSensorExtra`

```cpp
PieceSensorExtra
```

a piece sensor port on a mechanism with no distance or optical one

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:171`](../../include/shulib/mechanism/wiring.hpp#L171).*

<a id="entryproblem-pieceswitchunplugged"></a>

### `EntryProblem::PieceSwitchUnplugged`

```cpp
PieceSwitchUnplugged
```

the limit-switch piece sensor the mechanism has is unbound

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:172`](../../include/shulib/mechanism/wiring.hpp#L172).*

<a id="entryproblem-pieceswitchextra"></a>

### `EntryProblem::PieceSwitchExtra`

```cpp
PieceSwitchExtra
```

a piece switch line on a mechanism with no switch piece sensor

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:173`](../../include/shulib/mechanism/wiring.hpp#L173).*

<a id="entryproblem"></a>

## `entryProblem`

```cpp
[[nodiscard]] constexpr EntryProblem entryProblem(const MechanismSpec& s, const MechanismWiring& w) noexcept
```

THE PER-ENTRY WIRING RULES (header): the FIRST thing wrong with entry `w` beside its spec `s` — its name, then its motor roles, its line roles, a pneumatic's levels against its lines, its angle sensor, its home switch, its piece sensor — or EntryProblem::None. Every bound port is checked in range here, so a caller may index by it afterwards. It does not check the spec (specRefusal does).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:180`](../../include/shulib/mechanism/wiring.hpp#L180).*

<a id="entryproblemtext"></a>

## `entryProblemText`

```cpp
[[nodiscard]] constexpr const char* entryProblemText(EntryProblem e) noexcept
```

An entry problem as wiringProblem()'s SENTENCE (nullptr for None). An unbound or out-of-range role reads alike whichever role it is: "a smart-port role is unbound or not 1..21", "a 3-wire role is unbound or not A..H".

*free function, declared at [`include/shulib/mechanism/wiring.hpp:254`](../../include/shulib/mechanism/wiring.hpp#L254).*

<a id="wiringproblem"></a>

## `wiringProblem`

```cpp
[[nodiscard]] constexpr const char* wiringProblem(std::span<const MechanismSpec> specs, std::span<const MechanismWiring> wiring, std::span<const std::uint8_t> drivePorts, std::uint8_t imuPort) noexcept
```

THE WIRING CHECK at run time — the mirror of what wire() and mechanisms() refuse at build, for a table a TOOL or a test built by hand, plus the drive's ports (header): the first problem, or nullptr. Each entry is checked on its own first (entryProblem — the SAME rules mechanisms() holds a hand-built entry to), then against the others (a name or a port or line used twice) and the drive. `drivePorts` are the chassis table's port magnitudes, `imuPort` its IMU (0 = none). It checks the wiring, not the declarations (specProblem() does those), and not the route ids (routeIdsOf() does).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:295`](../../include/shulib/mechanism/wiring.hpp#L295).*

<a id="tomechanismconfig"></a>

## `toMechanismConfig`

```cpp
[[nodiscard]] constexpr config::MechanismConfig toMechanismConfig(const MechanismSpec& s, const MechanismWiring& w) noexcept
```

THE ONE PROJECTION: one mechanism as the device-level config robot::Robot (and the bench page) builds from — signed motor ports (a Reversed role is a NEGATIVE port, reversed once by its adapter), its lines, its sensor roles' ports and lines, and the start and safe LEVELS from its START and SAFE positions: one level for every line when each position is uniform across its lines (today's form — robot two's claw, byte for byte), per-line levels otherwise. PRECONDITION: specProblem(s) == nullptr and the wiring passed wiringProblem() (a mechanisms() list always has).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:389`](../../include/shulib/mechanism/wiring.hpp#L389).*

<a id="struct-motorports"></a>

## `struct MotorPorts`

```cpp
struct MotorPorts
```

The motor roles' smart ports, as motors(...) wrote them.

*struct, declared at [`include/shulib/mechanism/wiring.hpp:443`](../../include/shulib/mechanism/wiring.hpp#L443).*

<a id="motorports-port"></a>

### `MotorPorts::port`

```cpp
std::uint8_t port[kMaxMotorRoles]{}
```

The ports, in the declaration's `.motors` order.

*field, declared at [`include/shulib/mechanism/wiring.hpp:445`](../../include/shulib/mechanism/wiring.hpp#L445).*

<a id="motorports-count"></a>

### `MotorPorts::count`

```cpp
std::uint8_t count = 0
```

How many were written.

*field, declared at [`include/shulib/mechanism/wiring.hpp:447`](../../include/shulib/mechanism/wiring.hpp#L447).*

<a id="motorports-bad"></a>

### `MotorPorts::bad`

```cpp
bool bad = false
```

A number outside 1..21 was written.

*field, declared at [`include/shulib/mechanism/wiring.hpp:449`](../../include/shulib/mechanism/wiring.hpp#L449).*

<a id="motorports-file"></a>

### `MotorPorts::file`

```cpp
const char* file = nullptr
```

The file of the wire(...) line (captured where motors(...) is written).

*field, declared at [`include/shulib/mechanism/wiring.hpp:451`](../../include/shulib/mechanism/wiring.hpp#L451).*

<a id="motorports-line"></a>

### `MotorPorts::line`

```cpp
std::uint32_t line = 0
```

That line.

*field, declared at [`include/shulib/mechanism/wiring.hpp:453`](../../include/shulib/mechanism/wiring.hpp#L453).*

<a id="struct-lineports"></a>

## `struct LinePorts`

```cpp
struct LinePorts
```

A pneumatic's 3-wire lines, as lines(...) wrote them.

*struct, declared at [`include/shulib/mechanism/wiring.hpp:456`](../../include/shulib/mechanism/wiring.hpp#L456).*

<a id="lineports-line"></a>

### `LinePorts::line`

```cpp
Line line[kMaxLineRoles]{}
```

The lines, in the positions' level order.

*field, declared at [`include/shulib/mechanism/wiring.hpp:458`](../../include/shulib/mechanism/wiring.hpp#L458).*

<a id="lineports-count"></a>

### `LinePorts::count`

```cpp
std::uint8_t count = 0
```

How many were written.

*field, declared at [`include/shulib/mechanism/wiring.hpp:460`](../../include/shulib/mechanism/wiring.hpp#L460).*

<a id="lineports-bad"></a>

### `LinePorts::bad`

```cpp
bool bad = false
```

Not a letter 'A'..'H' was written.

*field, declared at [`include/shulib/mechanism/wiring.hpp:462`](../../include/shulib/mechanism/wiring.hpp#L462).*

<a id="lineports-file"></a>

### `LinePorts::file`

```cpp
const char* file = nullptr
```

The file of the wire(...) line (captured where lines(...) is written).

*field, declared at [`include/shulib/mechanism/wiring.hpp:464`](../../include/shulib/mechanism/wiring.hpp#L464).*

<a id="lineports-lineno"></a>

### `LinePorts::lineNo`

```cpp
std::uint32_t lineNo = 0
```

That line.

*field, declared at [`include/shulib/mechanism/wiring.hpp:466`](../../include/shulib/mechanism/wiring.hpp#L466).*

<a id="a_motor_port_is_a_number_like_6_a_letter_in_quotes_is_a_three_wire_port"></a>

## `a_motor_port_is_a_number_like_6_a_letter_in_quotes_is_a_three_wire_port`

```cpp
inline void a_motor_port_is_a_number_like_6_a_letter_in_quotes_is_a_three_wire_port()
```

motors('A'): a motor port is a number.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:473`](../../include/shulib/mechanism/wiring.hpp#L473).*

<a id="a_three_wire_port_is_a_letter_in_quotes_like_a_a_number_is_a_smart_port"></a>

## `a_three_wire_port_is_a_letter_in_quotes_like_A_a_number_is_a_smart_port`

```cpp
inline void a_three_wire_port_is_a_letter_in_quotes_like_A_a_number_is_a_smart_port()
```

lines(6, 7) / homeSwitch(66): a 3-wire port is a letter.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:475`](../../include/shulib/mechanism/wiring.hpp#L475).*

<a id="an_angle_sensor_is_a_smart_sensor_give_it_a_port_number"></a>

## `an_angle_sensor_is_a_smart_sensor_give_it_a_port_number`

```cpp
inline void an_angle_sensor_is_a_smart_sensor_give_it_a_port_number()
```

angleSensor('B'): an angle sensor is a smart sensor.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:477`](../../include/shulib/mechanism/wiring.hpp#L477).*

<a id="a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors"></a>

## `a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors`

```cpp
inline void a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors()
```

A smart port outside 1..21.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:479`](../../include/shulib/mechanism/wiring.hpp#L479).*

<a id="a_three_wire_port_is_not_a_letter_from_a_to_h_or_more_than_two_lines"></a>

## `a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines`

```cpp
inline void a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines()
```

A 3-wire port that is not 'A'..'H'.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:481`](../../include/shulib/mechanism/wiring.hpp#L481).*

<a id="the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has"></a>

## `the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has`

```cpp
inline void the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has()
```

motors(...) gives a different count than the declaration's `.motors`.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:483`](../../include/shulib/mechanism/wiring.hpp#L483).*

<a id="the_mechanism_has_a_sensor_the_wiring_does_not_plug_in"></a>

## `the_mechanism_has_a_sensor_the_wiring_does_not_plug_in`

```cpp
inline void the_mechanism_has_a_sensor_the_wiring_does_not_plug_in()
```

A sensor the declaration has, unplugged in the wiring.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:485`](../../include/shulib/mechanism/wiring.hpp#L485).*

<a id="the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have"></a>

## `the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have`

```cpp
inline void the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have()
```

A sensor plugged in that the declaration does not have.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:487`](../../include/shulib/mechanism/wiring.hpp#L487).*

<a id="a_piece_sensor_is_a_smart_sensor_give_it_a_port_number"></a>

## `a_piece_sensor_is_a_smart_sensor_give_it_a_port_number`

```cpp
inline void a_piece_sensor_is_a_smart_sensor_give_it_a_port_number()
```

pieceSensor('C') for a distance or optical sensor: it is a smart sensor.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:489`](../../include/shulib/mechanism/wiring.hpp#L489).*

<a id="a_limit_switch_is_a_three_wire_device_give_it_a_letter"></a>

## `a_limit_switch_is_a_three_wire_device_give_it_a_letter`

```cpp
inline void a_limit_switch_is_a_three_wire_device_give_it_a_letter()
```

pieceSensor(9) for a limit switch: it is a 3-wire device.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:491`](../../include/shulib/mechanism/wiring.hpp#L491).*

<a id="a_smart_port_is_used_twice"></a>

## `a_smart_port_is_used_twice`

```cpp
inline void a_smart_port_is_used_twice()
```

A smart port used twice — the fallback of the named list below; UNREACHABLE from a list (mechanisms()' per-entry check refuses a port outside 1..21 before it counts one), kept as the switch's default.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:495`](../../include/shulib/mechanism/wiring.hpp#L495).*

<a id="a_three_wire_port_is_used_twice"></a>

## `a_three_wire_port_is_used_twice`

```cpp
inline void a_three_wire_port_is_used_twice()
```

A 3-wire port used twice — the fallback of the named list below; UNREACHABLE from a list (mechanisms() skips an unbound line, and its per-entry check refuses one past H before it counts one).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:499`](../../include/shulib/mechanism/wiring.hpp#L499).*

<a id="the_wiring_plugs_in_the_same_sensor_twice"></a>

## `the_wiring_plugs_in_the_same_sensor_twice`

```cpp
inline void the_wiring_plugs_in_the_same_sensor_twice()
```

`wire(intake, motors(5), pieceSensor(9), pieceSensor(10))`: one sensor role written twice in a wire(...) line — the last would win and the first port would never be checked, counted or built (MECH-A1's review: a re-cabled sensor with its old line left in).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:503`](../../include/shulib/mechanism/wiring.hpp#L503).*

<a id="the_wiring_names_another_mechanism_than_its_declaration"></a>

## `the_wiring_names_another_mechanism_than_its_declaration`

```cpp
inline void the_wiring_names_another_mechanism_than_its_declaration()
```

A HAND-BUILT entry whose wiring names another mechanism than its declaration (wire() copies the name, so only an entry built by hand can).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:506`](../../include/shulib/mechanism/wiring.hpp#L506).*

<a id="the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has"></a>

## `the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has`

```cpp
inline void the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has()
```

A HAND-BUILT entry with a line role unbound, or a line on a role its declaration does not have (wire() makes the spec's line count from lines(...), so only an entry built by hand can).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:509`](../../include/shulib/mechanism/wiring.hpp#L509).*

<a id="more_mechanisms_than_a_robot_can_have"></a>

## `more_mechanisms_than_a_robot_can_have`

```cpp
inline void more_mechanisms_than_a_robot_can_have()
```

More than kMaxMechanisms in one list.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:511`](../../include/shulib/mechanism/wiring.hpp#L511).*

<a id="the_same_mechanism_is_wired_twice"></a>

## `the_same_mechanism_is_wired_twice`

```cpp
inline void the_same_mechanism_is_wired_twice()
```

The same declaration wired twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:513`](../../include/shulib/mechanism/wiring.hpp#L513).*

<a id="two_mechanisms_share_a_name"></a>

## `two_mechanisms_share_a_name`

```cpp
inline void two_mechanisms_share_a_name()
```

Two declarations with one `.name` (the run-time lookup is by name).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:515`](../../include/shulib/mechanism/wiring.hpp#L515).*

<a id="a_position_states_a_different_number_of_levels_than_the_wiring_has_lines"></a>

## `a_position_states_a_different_number_of_levels_than_the_wiring_has_lines`

```cpp
inline void a_position_states_a_different_number_of_levels_than_the_wiring_has_lines()
```

A position whose level count is not the wiring's line count (U2).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:517`](../../include/shulib/mechanism/wiring.hpp#L517).*

<a id="a_pneumatic_is_wired_with_lines_like_lines_a_not_motors"></a>

## `a_pneumatic_is_wired_with_lines_like_lines_A_not_motors`

```cpp
inline void a_pneumatic_is_wired_with_lines_like_lines_A_not_motors()
```

wire(pneumatic, motors(...)).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:519`](../../include/shulib/mechanism/wiring.hpp#L519).*

<a id="a_motor_mechanism_is_wired_with_motors_like_motors_6_not_lines"></a>

## `a_motor_mechanism_is_wired_with_motors_like_motors_6_not_lines`

```cpp
inline void a_motor_mechanism_is_wired_with_motors_like_motors_6_not_lines()
```

wire(arm, lines(...)).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:521`](../../include/shulib/mechanism/wiring.hpp#L521).*

<a id="smart_port_1_is_used_twice"></a>

## `smart_port_1_is_used_twice`

```cpp
inline void smart_port_1_is_used_twice()
```

Smart port 1 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:523`](../../include/shulib/mechanism/wiring.hpp#L523).*

<a id="smart_port_2_is_used_twice"></a>

## `smart_port_2_is_used_twice`

```cpp
inline void smart_port_2_is_used_twice()
```

Smart port 2 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:525`](../../include/shulib/mechanism/wiring.hpp#L525).*

<a id="smart_port_3_is_used_twice"></a>

## `smart_port_3_is_used_twice`

```cpp
inline void smart_port_3_is_used_twice()
```

Smart port 3 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:527`](../../include/shulib/mechanism/wiring.hpp#L527).*

<a id="smart_port_4_is_used_twice"></a>

## `smart_port_4_is_used_twice`

```cpp
inline void smart_port_4_is_used_twice()
```

Smart port 4 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:529`](../../include/shulib/mechanism/wiring.hpp#L529).*

<a id="smart_port_5_is_used_twice"></a>

## `smart_port_5_is_used_twice`

```cpp
inline void smart_port_5_is_used_twice()
```

Smart port 5 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:531`](../../include/shulib/mechanism/wiring.hpp#L531).*

<a id="smart_port_6_is_used_twice"></a>

## `smart_port_6_is_used_twice`

```cpp
inline void smart_port_6_is_used_twice()
```

Smart port 6 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:533`](../../include/shulib/mechanism/wiring.hpp#L533).*

<a id="smart_port_7_is_used_twice"></a>

## `smart_port_7_is_used_twice`

```cpp
inline void smart_port_7_is_used_twice()
```

Smart port 7 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:535`](../../include/shulib/mechanism/wiring.hpp#L535).*

<a id="smart_port_8_is_used_twice"></a>

## `smart_port_8_is_used_twice`

```cpp
inline void smart_port_8_is_used_twice()
```

Smart port 8 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:537`](../../include/shulib/mechanism/wiring.hpp#L537).*

<a id="smart_port_9_is_used_twice"></a>

## `smart_port_9_is_used_twice`

```cpp
inline void smart_port_9_is_used_twice()
```

Smart port 9 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:539`](../../include/shulib/mechanism/wiring.hpp#L539).*

<a id="smart_port_10_is_used_twice"></a>

## `smart_port_10_is_used_twice`

```cpp
inline void smart_port_10_is_used_twice()
```

Smart port 10 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:541`](../../include/shulib/mechanism/wiring.hpp#L541).*

<a id="smart_port_11_is_used_twice"></a>

## `smart_port_11_is_used_twice`

```cpp
inline void smart_port_11_is_used_twice()
```

Smart port 11 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:543`](../../include/shulib/mechanism/wiring.hpp#L543).*

<a id="smart_port_12_is_used_twice"></a>

## `smart_port_12_is_used_twice`

```cpp
inline void smart_port_12_is_used_twice()
```

Smart port 12 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:545`](../../include/shulib/mechanism/wiring.hpp#L545).*

<a id="smart_port_13_is_used_twice"></a>

## `smart_port_13_is_used_twice`

```cpp
inline void smart_port_13_is_used_twice()
```

Smart port 13 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:547`](../../include/shulib/mechanism/wiring.hpp#L547).*

<a id="smart_port_14_is_used_twice"></a>

## `smart_port_14_is_used_twice`

```cpp
inline void smart_port_14_is_used_twice()
```

Smart port 14 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:549`](../../include/shulib/mechanism/wiring.hpp#L549).*

<a id="smart_port_15_is_used_twice"></a>

## `smart_port_15_is_used_twice`

```cpp
inline void smart_port_15_is_used_twice()
```

Smart port 15 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:551`](../../include/shulib/mechanism/wiring.hpp#L551).*

<a id="smart_port_16_is_used_twice"></a>

## `smart_port_16_is_used_twice`

```cpp
inline void smart_port_16_is_used_twice()
```

Smart port 16 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:553`](../../include/shulib/mechanism/wiring.hpp#L553).*

<a id="smart_port_17_is_used_twice"></a>

## `smart_port_17_is_used_twice`

```cpp
inline void smart_port_17_is_used_twice()
```

Smart port 17 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:555`](../../include/shulib/mechanism/wiring.hpp#L555).*

<a id="smart_port_18_is_used_twice"></a>

## `smart_port_18_is_used_twice`

```cpp
inline void smart_port_18_is_used_twice()
```

Smart port 18 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:557`](../../include/shulib/mechanism/wiring.hpp#L557).*

<a id="smart_port_19_is_used_twice"></a>

## `smart_port_19_is_used_twice`

```cpp
inline void smart_port_19_is_used_twice()
```

Smart port 19 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:559`](../../include/shulib/mechanism/wiring.hpp#L559).*

<a id="smart_port_20_is_used_twice"></a>

## `smart_port_20_is_used_twice`

```cpp
inline void smart_port_20_is_used_twice()
```

Smart port 20 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:561`](../../include/shulib/mechanism/wiring.hpp#L561).*

<a id="smart_port_21_is_used_twice"></a>

## `smart_port_21_is_used_twice`

```cpp
inline void smart_port_21_is_used_twice()
```

Smart port 21 used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:563`](../../include/shulib/mechanism/wiring.hpp#L563).*

<a id="three_wire_port_a_is_used_twice"></a>

## `three_wire_port_A_is_used_twice`

```cpp
inline void three_wire_port_A_is_used_twice()
```

3-wire port A used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:565`](../../include/shulib/mechanism/wiring.hpp#L565).*

<a id="three_wire_port_b_is_used_twice"></a>

## `three_wire_port_B_is_used_twice`

```cpp
inline void three_wire_port_B_is_used_twice()
```

3-wire port B used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:567`](../../include/shulib/mechanism/wiring.hpp#L567).*

<a id="three_wire_port_c_is_used_twice"></a>

## `three_wire_port_C_is_used_twice`

```cpp
inline void three_wire_port_C_is_used_twice()
```

3-wire port C used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:569`](../../include/shulib/mechanism/wiring.hpp#L569).*

<a id="three_wire_port_d_is_used_twice"></a>

## `three_wire_port_D_is_used_twice`

```cpp
inline void three_wire_port_D_is_used_twice()
```

3-wire port D used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:571`](../../include/shulib/mechanism/wiring.hpp#L571).*

<a id="three_wire_port_e_is_used_twice"></a>

## `three_wire_port_E_is_used_twice`

```cpp
inline void three_wire_port_E_is_used_twice()
```

3-wire port E used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:573`](../../include/shulib/mechanism/wiring.hpp#L573).*

<a id="three_wire_port_f_is_used_twice"></a>

## `three_wire_port_F_is_used_twice`

```cpp
inline void three_wire_port_F_is_used_twice()
```

3-wire port F used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:575`](../../include/shulib/mechanism/wiring.hpp#L575).*

<a id="three_wire_port_g_is_used_twice"></a>

## `three_wire_port_G_is_used_twice`

```cpp
inline void three_wire_port_G_is_used_twice()
```

3-wire port G used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:577`](../../include/shulib/mechanism/wiring.hpp#L577).*

<a id="three_wire_port_h_is_used_twice"></a>

## `three_wire_port_H_is_used_twice`

```cpp
inline void three_wire_port_H_is_used_twice()
```

3-wire port H used twice.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:579`](../../include/shulib/mechanism/wiring.hpp#L579).*

<a id="motors"></a>

## `motors`

```cpp
template <class A> [[nodiscard]] consteval MotorPorts motors(A a, std::source_location at = std::source_location::current())
```

One mechanism's motor ports: `motors(6)` — a NUMBER per motor role, in its `.motors` order (one to four); `motors('A')` is refused by name. Records the line it is written on (the wire line).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:623`](../../include/shulib/mechanism/wiring.hpp#L623).*

<a id="motors-2"></a>

## `motors (overload 2)`

```cpp
template <class A, class B> [[nodiscard]] consteval MotorPorts motors(A a, B b, std::source_location at = std::source_location::current())
```

Two motor ports: `motors(2, 3)`.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:630`](../../include/shulib/mechanism/wiring.hpp#L630).*

<a id="motors-3"></a>

## `motors (overload 3)`

```cpp
template <class A, class B, class C> [[nodiscard]] consteval MotorPorts motors(A a, B b, C c, std::source_location at = std::source_location::current())
```

Three motor ports.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:638`](../../include/shulib/mechanism/wiring.hpp#L638).*

<a id="motors-4"></a>

## `motors (overload 4)`

```cpp
template <class A, class B, class C, class D> [[nodiscard]] consteval MotorPorts motors(A a, B b, C c, D d, std::source_location at = std::source_location::current())
```

Four motor ports (the most).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:647`](../../include/shulib/mechanism/wiring.hpp#L647).*

<a id="motors-5"></a>

## `motors (overload 5)`

```cpp
template <class A, class B, class C, class D, class E, class... More> consteval MotorPorts motors(A, B, C, D, E, More...)
```

FIVE or more motor ports: refused BY NAME — a mechanism has at most four motor roles (MECH-A1's review: without this overload it was a plain "no matching function" error).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:658`](../../include/shulib/mechanism/wiring.hpp#L658).*

<a id="motors-6"></a>

## `motors (overload 6)`

```cpp
[[nodiscard]] consteval MotorPorts motors(std::source_location at = std::source_location::current())
```

NO motor port, `motors()`: wire() then refuses it BY NAME, "the wiring gives a different number of motors than the mechanism has" (MECH-A1's review: no longer a plain C++ error).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:664`](../../include/shulib/mechanism/wiring.hpp#L664).*

<a id="lines"></a>

## `lines`

```cpp
template <class A> [[nodiscard]] consteval LinePorts lines(A a, std::source_location at = std::source_location::current())
```

A pneumatic mechanism's 3-wire line: `lines('A')` — a LETTER in quotes; `lines(6)` is refused by name. Records the line it is written on.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:670`](../../include/shulib/mechanism/wiring.hpp#L670).*

<a id="lines-2"></a>

## `lines (overload 2)`

```cpp
template <class A, class B> [[nodiscard]] consteval LinePorts lines(A a, B b, std::source_location at = std::source_location::current())
```

Two lines, in its positions' level order: `lines('C', 'D')`.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:677`](../../include/shulib/mechanism/wiring.hpp#L677).*

<a id="lines-3"></a>

## `lines (overload 3)`

```cpp
template <class A, class B, class C, class... More> consteval LinePorts lines(A, B, C, More...)
```

THREE or more lines: refused BY NAME — a pneumatic mechanism has at most two line roles (MECH-A1's review: without this overload it was a plain "no matching function" error).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:686`](../../include/shulib/mechanism/wiring.hpp#L686).*

<a id="lines-4"></a>

## `lines (overload 4)`

```cpp
[[nodiscard]] consteval LinePorts lines(std::source_location at = std::source_location::current())
```

NO line, `lines()`: wire() then refuses it BY NAME, "a pneumatic mechanism needs one or two line roles" (MECH-A1's review: no longer a plain C++ error).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:692`](../../include/shulib/mechanism/wiring.hpp#L692).*

<a id="struct-roleport"></a>

## `struct RolePort`

```cpp
struct RolePort
```

A SENSOR role's port, as angleSensor(10), pieceSensor(9) / pieceSensor('E'), homeSwitch('B') wrote it.

*struct, declared at [`include/shulib/mechanism/wiring.hpp:698`](../../include/shulib/mechanism/wiring.hpp#L698).*

<a id="roleport-port"></a>

### `RolePort::port`

```cpp
std::uint8_t port = 0
```

Its smart port; 0 for a line.

*field, declared at [`include/shulib/mechanism/wiring.hpp:706`](../../include/shulib/mechanism/wiring.hpp#L706).*

<a id="roleport-line"></a>

### `RolePort::line`

```cpp
Line line = Line::None
```

Its line; None for a smart port.

*field, declared at [`include/shulib/mechanism/wiring.hpp:708`](../../include/shulib/mechanism/wiring.hpp#L708).*

<a id="roleport-letter"></a>

### `RolePort::letter`

```cpp
bool letter = false
```

Written as a LETTER (so a bad value is a bad 3-wire port, not a bad smart port).

*field, declared at [`include/shulib/mechanism/wiring.hpp:710`](../../include/shulib/mechanism/wiring.hpp#L710).*

<a id="roleport-bad"></a>

### `RolePort::bad`

```cpp
bool bad = false
```

Out of range.

*field, declared at [`include/shulib/mechanism/wiring.hpp:712`](../../include/shulib/mechanism/wiring.hpp#L712).*

<a id="enum-class-roleport-role"></a>

## `enum class RolePort::Role`

```cpp
enum class Role : std::uint8_t
```

Which role it is.

*enum class, declared at [`include/shulib/mechanism/wiring.hpp:700`](../../include/shulib/mechanism/wiring.hpp#L700).*

<a id="roleport-role-angle"></a>

### `RolePort::Role::Angle`

```cpp
Angle
```

the angle sensor (a smart port)

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:701`](../../include/shulib/mechanism/wiring.hpp#L701).*

<a id="roleport-role-piece"></a>

### `RolePort::Role::Piece`

```cpp
Piece
```

the piece sensor (a smart port, or a line for a limit switch)

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:702`](../../include/shulib/mechanism/wiring.hpp#L702).*

<a id="roleport-role-homeswitch"></a>

### `RolePort::Role::HomeSwitch`

```cpp
HomeSwitch
```

the home switch (a line)

*enumerator, declared at [`include/shulib/mechanism/wiring.hpp:703`](../../include/shulib/mechanism/wiring.hpp#L703).*

<a id="anglesensor"></a>

## `angleSensor`

```cpp
template <class P> [[nodiscard]] consteval RolePort angleSensor(P port)
```

The angle sensor's smart port: `angleSensor(10)` — a NUMBER; `angleSensor('B')` is refused by name.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:716`](../../include/shulib/mechanism/wiring.hpp#L716).*

<a id="piecesensor"></a>

## `pieceSensor`

```cpp
template <class P> [[nodiscard]] consteval RolePort pieceSensor(P port)
```

The piece sensor's port: `pieceSensor(9)` for a distance or optical sensor (a number), `pieceSensor('E')` for a limit switch (a letter) — the declaration says which, and wire() refuses the wrong form by name.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:730`](../../include/shulib/mechanism/wiring.hpp#L730).*

<a id="homeswitch"></a>

## `homeSwitch`

```cpp
template <class C> [[nodiscard]] consteval RolePort homeSwitch(C line)
```

The home switch's line: `homeSwitch('B')` — a LETTER; `homeSwitch(66)` is refused by name.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:743`](../../include/shulib/mechanism/wiring.hpp#L743).*

<a id="struct-wired"></a>

## `struct Wired`

```cpp
struct Wired
```

One mechanism, checked and wired: its spec, its ports, and its declaration's address — what one wire(...) line makes.

*struct, declared at [`include/shulib/mechanism/wiring.hpp:835`](../../include/shulib/mechanism/wiring.hpp#L835).*

<a id="wired-spec"></a>

### `Wired::spec`

```cpp
MechanismSpec spec{}
```

The declaration as data.

*field, declared at [`include/shulib/mechanism/wiring.hpp:837`](../../include/shulib/mechanism/wiring.hpp#L837).*

<a id="wired-wiring"></a>

### `Wired::wiring`

```cpp
MechanismWiring wiring{}
```

Its ports.

*field, declared at [`include/shulib/mechanism/wiring.hpp:839`](../../include/shulib/mechanism/wiring.hpp#L839).*

<a id="wired-declared"></a>

### `Wired::declared`

```cpp
const void* declared = nullptr
```

The declaration's address.

*field, declared at [`include/shulib/mechanism/wiring.hpp:841`](../../include/shulib/mechanism/wiring.hpp#L841).*

<a id="wire"></a>

## `wire`

```cpp
template <class... R> [[nodiscard]] consteval Wired wire(const Power& d, MotorPorts m, R... r)
```

`wire(intake, motors(5), pieceSensor(9))`: one power mechanism's wiring, checked AT BUILD.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:931`](../../include/shulib/mechanism/wiring.hpp#L931).*

<a id="wire-2"></a>

## `wire (overload 2)`

```cpp
template <class... R> [[nodiscard]] consteval Wired wire(const Arm& d, MotorPorts m, R... r)
```

`wire(arm, motors(1))`: one arm's wiring, checked AT BUILD.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:936`](../../include/shulib/mechanism/wiring.hpp#L936).*

<a id="wire-3"></a>

## `wire (overload 3)`

```cpp
template <class... R> [[nodiscard]] consteval Wired wire(const Lift& d, MotorPorts m, R... r)
```

`wire(lift, motors(2, 3), homeSwitch('B'))`: one lift's wiring, checked AT BUILD.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:941`](../../include/shulib/mechanism/wiring.hpp#L941).*

<a id="wire-4"></a>

## `wire (overload 4)`

```cpp
template <class... R> [[nodiscard]] consteval Wired wire(const Flywheel& d, MotorPorts m, R... r)
```

`wire(flywheel, motors(4))`: one flywheel's wiring, checked AT BUILD.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:946`](../../include/shulib/mechanism/wiring.hpp#L946).*

<a id="wire-5"></a>

## `wire (overload 5)`

```cpp
template <class... R> [[nodiscard]] consteval Wired wire(const Pneumatic& d, LinePorts l, R... roles)
```

`wire(hood, lines('C', 'D'))`: one pneumatic's wiring, checked AT BUILD — every position's level count against the lines FIRST (U2: one level on two lines would leave the second cylinder dead in every position; checked before the spec, so the slip is named as itself).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:953`](../../include/shulib/mechanism/wiring.hpp#L953).*

<a id="wire-6"></a>

## `wire (overload 6)`

```cpp
template <class... R> consteval Wired wire(const Pneumatic&, MotorPorts, R...)
```

`wire(hood, motors(...))`: refused by name — a pneumatic is wired with lines.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:978`](../../include/shulib/mechanism/wiring.hpp#L978).*

<a id="wire-7"></a>

## `wire (overload 7)`

```cpp
template <class D, class... R> consteval Wired wire(const D&, LinePorts, R...)
```

`wire(arm, lines(...))`: refused by name — a motor kind is wired with motors.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:985`](../../include/shulib/mechanism/wiring.hpp#L985).*

<a id="mechanisms"></a>

## `mechanisms`

```cpp
template <class... W> [[nodiscard]] consteval MechanismTable mechanisms(const W&... wired)
```

THE ROBOT'S MECHANISMS: every wire(...) line, in boot order — checked AT BUILD across mechanisms (header): a port or a line used twice (named), the same declaration twice, two `.name`s alike, more than kMaxMechanisms, and every route id the list publishes (route_ids.hpp). Each entry is checked AGAIN on its own here — its spec (specRefusal) and its wiring beside it (entryProblem), by the names wire() uses — before any port is counted: wire() checks what a declaration makes, and an entry built by hand must not walk around it (MECH-A1's review found that it could).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:997`](../../include/shulib/mechanism/wiring.hpp#L997).*

<a id="mechanisms-2"></a>

## `mechanisms (overload 2)`

```cpp
[[nodiscard]] consteval MechanismTable mechanisms()
```

A robot with no mechanisms.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:1054`](../../include/shulib/mechanism/wiring.hpp#L1054).*

<a id="clearofdrive"></a>

## `clearOfDrive`

```cpp
[[nodiscard]] constexpr bool clearOfDrive(const MechanismTable& r, std::span<const std::uint8_t> drivePorts, std::uint8_t imuPort) noexcept
```

THE DRIVE CHECK (the robot's own typed chassis table, beside its list — MECH-A5's `mechanismsClearOf`): no mechanism role on a drive port or the IMU's — the build-time mirror of RobotConfig::valid(), one of the places a later PTO changes (the brief's §1.15).

*free function, declared at [`include/shulib/mechanism/wiring.hpp:1059`](../../include/shulib/mechanism/wiring.hpp#L1059).*

<a id="declaredmotorports"></a>

## `declaredMotorPorts`

```cpp
[[nodiscard]] constexpr std::size_t declaredMotorPorts(const MechanismTable& r, std::span<int> out) noexcept
```

Every bound mechanism MOTOR port, physical, in list order — what the drive's boot check is told as its declared motor ports (calibration/boot_check.hpp `DeclaredMotorPorts`). Writes at most `out.size()`; returns the count written.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:1067`](../../include/shulib/mechanism/wiring.hpp#L1067).*

<a id="roleline"></a>

## `roleLine`

```cpp
inline void roleLine(char* out, std::size_t cap, const char* mechanismName, const char* role, std::uint8_t port, Line line, RoleState st, const char* refusal, bool degradeOnly)
```

ONE BOOT LINE per role (tag MECH; the caller logs it), from what robot::MechanismParts recorded: "<mechanism>: <role> -> port <N> ok" "<mechanism>: <role> -> port <N> ABSENT: <the provider's sentence>" "<mechanism>: <role> -> port <N> MISSING, it runs WITHOUT it: <the provider's sentence>" "<mechanism>: <role> -> 3-wire <X> not checkable (the brain cannot see a 3-wire device)" "<mechanism>: <role> -> port <N> not asked (the mechanism is already ABSENT)" A 3-wire role names its line ("3-wire B") in every form. Truncates, never overflows; writes at least a NUL whenever `cap > 0`.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:1087`](../../include/shulib/mechanism/wiring.hpp#L1087).*

<a id="startingvaluesline"></a>

## `startingValuesLine`

```cpp
inline void startingValuesLine(char* out, std::size_t cap, const MechanismSpec& s)
```

ONE boot line naming the numbers a POSITION or SPEED mechanism's loop runs with, and which of them are the library's STARTING values (HA-182 — invented, never measured) rather than the team's `.tuning` — beside roleLine(). No program prints it yet: the core's fifth part (MECH-A5) adds it to every program's boot. Each number that differs from positionStartingValues() / speedStartingValues() is marked "(tuned)" — for the smooth move's ramp, its TIME (a stated speed alone keeps the starting 0.25 s ramp, so it is not marked); the line ends saying how many are starting values. Writes at most `cap` bytes, NUL-terminated; "" for a power or pneumatic mechanism (they run no loop). Verbatim, an untuned arm and flywheel: arm: starting values kP 0.250 kI 0.300 kD 0.0040 kV 0.0500, cruise 180.0 deg/s in 0.25 s, reached within 1.50 deg for 0.10 s -- PROVISIONAL (HA-182); .tuning overrides flywheel: starting values kV 0.0318 kP 0.050 (V per output rad/s) kI 0.200 (V per output rad) -- PROVISIONAL (HA-182); .tuning overrides (each one line). A number stated by `.tuning` prints its value and "(tuned)"; when every one is, the line ends "-- every number stated by its .tuning". The library's other HA-182 numbers — the hold's integral clamp, the speed integral's window and clamp (mechanism/service.hpp) — are constants, not `.tuning` fields, and are not on this line.

*free function, declared at [`include/shulib/mechanism/wiring.hpp:1136`](../../include/shulib/mechanism/wiring.hpp#L1136).*

<a id="mechanismtable"></a>

## `MechanismTable`

```cpp
using mechanism::MechanismTable
```

A robot's list of mechanisms: `inline constexpr MechanismTable kRobotXMechanisms = mechanisms(...)`.

*type alias, declared at [`include/shulib/mechanism/wiring.hpp:1203`](../../include/shulib/mechanism/wiring.hpp#L1203).*

<a id="mechanisms-3"></a>

## `mechanisms (overload 3)`

```cpp
using mechanism::mechanisms
```

The list itself.

*type alias, declared at [`include/shulib/mechanism/wiring.hpp:1205`](../../include/shulib/mechanism/wiring.hpp#L1205).*

<a id="wire-8"></a>

## `wire (overload 8)`

```cpp
using mechanism::wire
```

One mechanism's line of it.

*type alias, declared at [`include/shulib/mechanism/wiring.hpp:1207`](../../include/shulib/mechanism/wiring.hpp#L1207).*

<a id="motors-7"></a>

## `motors (overload 7)`

```cpp
using mechanism::motors
```

Its motors' smart ports.

*type alias, declared at [`include/shulib/mechanism/wiring.hpp:1209`](../../include/shulib/mechanism/wiring.hpp#L1209).*

<a id="lines-5"></a>

## `lines (overload 5)`

```cpp
using mechanism::lines
```

Its 3-wire lines.

*type alias, declared at [`include/shulib/mechanism/wiring.hpp:1211`](../../include/shulib/mechanism/wiring.hpp#L1211).*

<a id="anglesensor-2"></a>

## `angleSensor (overload 2)`

```cpp
using mechanism::angleSensor
```

Its angle sensor's smart port.

*type alias, declared at [`include/shulib/mechanism/wiring.hpp:1213`](../../include/shulib/mechanism/wiring.hpp#L1213).*

<a id="piecesensor-2"></a>

## `pieceSensor (overload 2)`

```cpp
using mechanism::pieceSensor
```

Its piece sensor's port (a number) or line (a letter).

*type alias, declared at [`include/shulib/mechanism/wiring.hpp:1215`](../../include/shulib/mechanism/wiring.hpp#L1215).*

<a id="homeswitch-2"></a>

## `homeSwitch (overload 2)`

```cpp
using mechanism::homeSwitch
```

Its home switch's line.

*type alias, declared at [`include/shulib/mechanism/wiring.hpp:1217`](../../include/shulib/mechanism/wiring.hpp#L1217).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 49 lines, click to expand</summary>

```text

 mechanism/wiring.hpp — THE BINDING (the mechanism core, chunk MECH-A1, 2026-10-08): which port
 each ROLE is plugged into, a block of the robot file SEPARATE from the declarations, compiled in
 (P2; the team lead's Q1 (a): the binding lives only in the robot file).

     inline constexpr MechanismTable kRobotXMechanisms = mechanisms(
         wire(arm, motors(1)),
         wire(lift, motors(2, 3), homeSwitch('B')),
         wire(flywheel, motors(4)),
         wire(intake, motors(5), pieceSensor(9)),
         wire(hood, lines('C', 'D')));

 One line per mechanism, in the order the boot lists them; this list IS the robot's mechanisms (a
 robot names its list `k<Robot>Mechanisms` — `MechanismTable` is the library's one type name, and
 `kMechanisms` is the BUILD's selection, never a robot's). A smart port is a NUMBER (1..21), a
 3-wire port a LETTER in quotes ('A'..'H') — `Line` is its own type, so smart port 6 and line F
 cannot be confused (an early census probe read "claw line = 6" as 3-wire F).

 ── Checked AT BUILD, by name ────────────────────────────────────────────────────────
 Each wire() checks ITS mechanism: the declaration itself (spec.hpp's specRefusal), every role
 bound and nothing extra, no sensor role written twice, a port in range and of the right KIND (a
 number where a smart device goes, a letter where a 3-wire one does), the motor count (at most
 four) and the line count (one or two), and — before the spec's own checks — that every pneumatic
 position states one level per wired line (U2). mechanisms() checks the ROBOT: each entry again on
 its own (its spec, and its wiring beside it — entryProblem(), the rules wiringProblem() shares —
 so an entry built by hand is held to wire()'s rules), then no smart port or 3-wire port used
 twice — NAMING it (`smart_port_4_is_used_twice`, `three_wire_port_C_is_used_twice`) —, the same
 declaration twice, two declarations sharing a `.name`, more than eight, and every route id the
 list would publish (route_ids.hpp). Each refusal stops the build in a consteval function whose
 NAME is the reason (spec.hpp's `refused::` idiom).
 Where GCC puts it, measured (the brief's neg5 set): a refusal between entries points at the
 list's first line and names the port; a declaration's own refusal points at its wire(...) line.

 ── The drive check, and the run-time owner of every collision ───────────────────────
 The drive's ports and the IMU's are checked against the robot's own chassis table beside its list
 (clearOfDrive, wiringProblem — the robot table file's job, MECH-A5) and, at RUN time, by
 config::RobotConfig::valid() over the derived config (toMechanismConfig) beside the drivetrain the
 boot actually chose — a calibration record's ports included. valid() is the owner; the build-time
 checks mirror it for the typed drive, and the host suite proves the two agree.

 ── At every boot ───────────────────────────────────────────────────────────────────
 robot::MechanismParts asks the registry-checked device provider for every role's device (the
 provider IS the kind check, HA-141). A missing or wrong-kind ACTUATOR or ANGLE sensor makes that
 mechanism ABSENT (its controls and steps do nothing; the drive runs); a missing PIECE sensor leaves
 it running WITHOUT one. roleLine() words one boot line per role; a 3-wire role reads "not
 checkable" (the brain cannot see a 3-wire device, HA-158). Two cables of the SAME kind swapped
 between mechanisms look fine at boot — the bench page's direction check is for that (MECH-A6).

 PROS-free, allocation-free; the list is constexpr (flash on the brain).
```

</details>
