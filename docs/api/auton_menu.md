<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/auton_menu.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `auton_menu.hpp`

competition::AutonMenu — the brain-screen auton selector as a PURE MODEL.

This header declares **7** types (46 members) and **16** constants.

Extracted from [`include/shulib/competition/auton_menu.hpp`](../../include/shulib/competition/auton_menu.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMenuEntriesPerPage`](#kmenuentriesperpage) — *constant*
- [`kMenuMaxButtons`](#kmenumaxbuttons) — *constant*
- [`kMenuHeaderBar`](#kmenuheaderbar) — *constant*
- [`kMenuEntryRow0`](#kmenuentryrow0) — *constant*
- [`kMenuEntryRow1`](#kmenuentryrow1) — *constant*
- [`kMenuPrevSlot`](#kmenuprevslot) — *constant*
- [`kMenuNoneSlot`](#kmenunoneslot) — *constant*
- [`kMenuNextSlot`](#kmenunextslot) — *constant*
- [`kMenuEntryRows`](#kmenuentryrows) — *constant*
- [`kMenuHeaderPrefix`](#kmenuheaderprefix) — *constant*
- [`kMenuNoneLabel`](#kmenunonelabel) — *constant*
- [`kMenuPrevLabel`](#kmenuprevlabel) — *constant*
- [`kMenuNextLabel`](#kmenunextlabel) — *constant*
- [`kMenuHintText`](#kmenuhinttext) — *constant*
- [`kMenuLockedText`](#kmenulockedtext) — *constant*
- [`kMenuNoneInfo`](#kmenunoneinfo) — *constant*
- [`enum class MenuButtonId`](#enum-class-menubuttonid)
  - [`Entry`](#menubuttonid-entry)
  - [`None`](#menubuttonid-none)
  - [`Prev`](#menubuttonid-prev)
  - [`Next`](#menubuttonid-next)
- [`struct MenuButton`](#struct-menubutton)
  - [`id`](#menubutton-id)
  - [`entry`](#menubutton-entry)
  - [`rect`](#menubutton-rect)
  - [`1]`](#menubutton-1)
  - [`armed`](#menubutton-armed)
  - [`chosen`](#menubutton-chosen)
- [`struct MenuLine`](#struct-menuline)
  - [`x`](#menuline-x)
  - [`y`](#menuline-y)
  - [`size`](#menuline-size)
  - [`tone`](#menuline-tone)
  - [`1]`](#menuline-1)
- [`struct MenuView`](#struct-menuview)
  - [`header`](#menuview-header)
  - [`info`](#menuview-info)
  - [`status`](#menuview-status)
  - [`buttons`](#menuview-buttons)
  - [`buttonCount`](#menuview-buttoncount)
  - [`page`](#menuview-page)
  - [`pageCount`](#menuview-pagecount)
  - [`locked`](#menuview-locked)
- [`enum class MenuTouchAction`](#enum-class-menutouchaction)
  - [`None`](#menutouchaction-none)
  - [`Chose`](#menutouchaction-chose)
  - [`Paged`](#menutouchaction-paged)
  - [`RefusedLocked`](#menutouchaction-refusedlocked)
- [`struct MenuTouch`](#struct-menutouch)
  - [`action`](#menutouch-action)
  - [`hit`](#menutouch-hit)
  - [`newRelease`](#menutouch-newrelease)
  - [`choice`](#menutouch-choice)
- [`class AutonMenu`](#class-autonmenu)
  - [`AutonMenu`](#autonmenu-autonmenu)
  - [`AutonMenu (overload 2)`](#autonmenu-autonmenu-2)
  - [`operator=`](#autonmenu-operator-eq)
  - [`AutonMenu (overload 3)`](#autonmenu-autonmenu-3)
  - [`operator= (overload 2)`](#autonmenu-operator-eq-2)
  - [`~AutonMenu`](#autonmenu-destructor-autonmenu)
  - [`view`](#autonmenu-view)
  - [`touch`](#autonmenu-touch)
  - [`choice`](#autonmenu-choice)
  - [`chosenEntry`](#autonmenu-chosenentry)
  - [`lockChoice`](#autonmenu-lockchoice)
  - [`unlock`](#autonmenu-unlock)
  - [`locked`](#autonmenu-locked)
  - [`page`](#autonmenu-page)
  - [`pageCount`](#autonmenu-pagecount)

<a id="kmenuentriesperpage"></a>

## `kMenuEntriesPerPage`

```cpp
inline constexpr std::size_t kMenuEntriesPerPage = 2
```

How many auton buttons one page shows. Two full-width rows fit between the two status lines and the bottom row with every vertical gap above 2 * ui::kHitPadY (the layout rule that keeps two padded hit boxes from overlapping). The layout is an ESTIMATE like every screen number (HA-136).

*constant, declared at [`include/shulib/competition/auton_menu.hpp:53`](../../include/shulib/competition/auton_menu.hpp#L53).*

<a id="kmenumaxbuttons"></a>

## `kMenuMaxButtons`

```cpp
inline constexpr std::size_t kMenuMaxButtons = kMenuEntriesPerPage + 3
```

The most buttons one view carries: the page's autons plus PREV, NONE and NEXT.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:55`](../../include/shulib/competition/auton_menu.hpp#L55).*

<a id="kmenuheaderbar"></a>

## `kMenuHeaderBar`

```cpp
inline constexpr ui::Rect kMenuHeaderBar{0, 0, 479, 26}
```

The header bar (the current choice).

*constant, declared at [`include/shulib/competition/auton_menu.hpp:58`](../../include/shulib/competition/auton_menu.hpp#L58).*

<a id="kmenuentryrow0"></a>

## `kMenuEntryRow0`

```cpp
inline constexpr ui::Rect kMenuEntryRow0{6, 66, 474, 100}
```

The first auton row: full width, so a 29-character name and its kind fit one medium line.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:60`](../../include/shulib/competition/auton_menu.hpp#L60).*

<a id="kmenuentryrow1"></a>

## `kMenuEntryRow1`

```cpp
inline constexpr ui::Rect kMenuEntryRow1{6, 122, 474, 156}
```

The second auton row.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:62`](../../include/shulib/competition/auton_menu.hpp#L62).*

<a id="kmenuprevslot"></a>

## `kMenuPrevSlot`

```cpp
inline constexpr ui::Rect kMenuPrevSlot{6, 180, 154, 234}
```

PREV, bottom left — drawn only when there is a previous page.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:64`](../../include/shulib/competition/auton_menu.hpp#L64).*

<a id="kmenunoneslot"></a>

## `kMenuNoneSlot`

```cpp
inline constexpr ui::Rect kMenuNoneSlot{166, 180, 314, 234}
```

NONE, bottom centre — drawn on EVERY view, always in this place.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:66`](../../include/shulib/competition/auton_menu.hpp#L66).*

<a id="kmenunextslot"></a>

## `kMenuNextSlot`

```cpp
inline constexpr ui::Rect kMenuNextSlot{326, 180, 474, 234}
```

NEXT, bottom right — drawn only when there is a next page.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:68`](../../include/shulib/competition/auton_menu.hpp#L68).*

<a id="kmenuentryrows"></a>

## `kMenuEntryRows`

```cpp
inline constexpr ui::Rect kMenuEntryRows[kMenuEntriesPerPage] = {kMenuEntryRow0, kMenuEntryRow1}
```

The auton rows, top to bottom.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:71`](../../include/shulib/competition/auton_menu.hpp#L71).*

<a id="kmenuheaderprefix"></a>

## `kMenuHeaderPrefix`

```cpp
inline constexpr std::string_view kMenuHeaderPrefix = "AUTON: "
```

The header's prefix; "AUTON: " + a kMaxAutonNameChars name is one medium line (B25).

*constant, declared at [`include/shulib/competition/auton_menu.hpp:74`](../../include/shulib/competition/auton_menu.hpp#L74).*

<a id="kmenunonelabel"></a>

## `kMenuNoneLabel`

```cpp
inline constexpr std::string_view kMenuNoneLabel = "NONE"
```

NONE's label.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:76`](../../include/shulib/competition/auton_menu.hpp#L76).*

<a id="kmenuprevlabel"></a>

## `kMenuPrevLabel`

```cpp
inline constexpr std::string_view kMenuPrevLabel = "< PREV"
```

PREV's label.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:78`](../../include/shulib/competition/auton_menu.hpp#L78).*

<a id="kmenunextlabel"></a>

## `kMenuNextLabel`

```cpp
inline constexpr std::string_view kMenuNextLabel = "NEXT >"
```

NEXT's label.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:80`](../../include/shulib/competition/auton_menu.hpp#L80).*

<a id="kmenuhinttext"></a>

## `kMenuHintText`

```cpp
inline constexpr std::string_view kMenuHintText = "touch a name to choose it; NONE runs nothing"
```

The status line while nothing is locked.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:82`](../../include/shulib/competition/auton_menu.hpp#L82).*

<a id="kmenulockedtext"></a>

## `kMenuLockedText`

```cpp
inline constexpr std::string_view kMenuLockedText = "LOCKED -- the auton has started"
```

The status line once the auton has started.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:84`](../../include/shulib/competition/auton_menu.hpp#L84).*

<a id="kmenunoneinfo"></a>

## `kMenuNoneInfo`

```cpp
inline constexpr std::string_view kMenuNoneInfo = "NONE: no auton runs (the default)"
```

The info line for NONE.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:86`](../../include/shulib/competition/auton_menu.hpp#L86).*

<a id="enum-class-menubuttonid"></a>

## `enum class MenuButtonId`

```cpp
enum class MenuButtonId : std::uint8_t
```

Which button.

*enum class, declared at [`include/shulib/competition/auton_menu.hpp:112`](../../include/shulib/competition/auton_menu.hpp#L112).*

<a id="menubuttonid-entry"></a>

### `MenuButtonId::Entry`

```cpp
Entry
```

a registered auton (MenuButton::entry says which)

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:113`](../../include/shulib/competition/auton_menu.hpp#L113).*

<a id="menubuttonid-none"></a>

### `MenuButtonId::None`

```cpp
None
```

NONE — always present, never delayed

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:114`](../../include/shulib/competition/auton_menu.hpp#L114).*

<a id="menubuttonid-prev"></a>

### `MenuButtonId::Prev`

```cpp
Prev
```

the previous page

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:115`](../../include/shulib/competition/auton_menu.hpp#L115).*

<a id="menubuttonid-next"></a>

### `MenuButtonId::Next`

```cpp
Next
```

the next page

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:116`](../../include/shulib/competition/auton_menu.hpp#L116).*

<a id="struct-menubutton"></a>

## `struct MenuButton`

```cpp
struct MenuButton
```

One button of a view.

*struct, declared at [`include/shulib/competition/auton_menu.hpp:120`](../../include/shulib/competition/auton_menu.hpp#L120).*

<a id="menubutton-id"></a>

### `MenuButton::id`

```cpp
MenuButtonId id = MenuButtonId::None
```

Which button.

*field, declared at [`include/shulib/competition/auton_menu.hpp:122`](../../include/shulib/competition/auton_menu.hpp#L122).*

<a id="menubutton-entry"></a>

### `MenuButton::entry`

```cpp
int entry = -1
```

For an Entry: the registry index it chooses. -1 otherwise.

*field, declared at [`include/shulib/competition/auton_menu.hpp:124`](../../include/shulib/competition/auton_menu.hpp#L124).*

<a id="menubutton-rect"></a>

### `MenuButton::rect`

```cpp
ui::Rect rect{}
```

Where it is drawn (its hit box is ui::hitBox(rect)).

*field, declared at [`include/shulib/competition/auton_menu.hpp:126`](../../include/shulib/competition/auton_menu.hpp#L126).*

<a id="menubutton-1"></a>

### `MenuButton::1]`

```cpp
char label[ui::kMediumCols + 1]{}
```

Its label, MEDIUM, screen-safe.

*field, declared at [`include/shulib/competition/auton_menu.hpp:128`](../../include/shulib/competition/auton_menu.hpp#L128).*

<a id="menubutton-armed"></a>

### `MenuButton::armed`

```cpp
bool armed = true
```

False while the arming delay runs or once the choice is locked: drawn GREYED, never hit.

*field, declared at [`include/shulib/competition/auton_menu.hpp:130`](../../include/shulib/competition/auton_menu.hpp#L130).*

<a id="menubutton-chosen"></a>

### `MenuButton::chosen`

```cpp
bool chosen = false
```

True for the button whose choice is the current one (drawn highlighted).

*field, declared at [`include/shulib/competition/auton_menu.hpp:132`](../../include/shulib/competition/auton_menu.hpp#L132).*

<a id="struct-menuline"></a>

## `struct MenuLine`

```cpp
struct MenuLine
```

One line of text at a fixed place and size.

*struct, declared at [`include/shulib/competition/auton_menu.hpp:136`](../../include/shulib/competition/auton_menu.hpp#L136).*

<a id="menuline-x"></a>

### `MenuLine::x`

```cpp
std::int16_t x = 0
```

Left edge, pixels.

*field, declared at [`include/shulib/competition/auton_menu.hpp:138`](../../include/shulib/competition/auton_menu.hpp#L138).*

<a id="menuline-y"></a>

### `MenuLine::y`

```cpp
std::int16_t y = 0
```

Top edge, pixels.

*field, declared at [`include/shulib/competition/auton_menu.hpp:140`](../../include/shulib/competition/auton_menu.hpp#L140).*

<a id="menuline-size"></a>

### `MenuLine::size`

```cpp
ui::TextSize size = ui::TextSize::Medium
```

Its size; the text fits ui::budgetFor(size).

*field, declared at [`include/shulib/competition/auton_menu.hpp:142`](../../include/shulib/competition/auton_menu.hpp#L142).*

<a id="menuline-tone"></a>

### `MenuLine::tone`

```cpp
ui::Tone tone = ui::Tone::White
```

Its colour, by meaning.

*field, declared at [`include/shulib/competition/auton_menu.hpp:144`](../../include/shulib/competition/auton_menu.hpp#L144).*

<a id="menuline-1"></a>

### `MenuLine::1]`

```cpp
char text[ui::kSmallCols + 1]{}
```

The text, NUL-terminated, screen-safe.

*field, declared at [`include/shulib/competition/auton_menu.hpp:146`](../../include/shulib/competition/auton_menu.hpp#L146).*

<a id="struct-menuview"></a>

## `struct MenuView`

```cpp
struct MenuView
```

Everything one frame of the selector says, and which touches mean something.

*struct, declared at [`include/shulib/competition/auton_menu.hpp:150`](../../include/shulib/competition/auton_menu.hpp#L150).*

<a id="menuview-header"></a>

### `MenuView::header`

```cpp
MenuLine header{}
```

"AUTON: <name>" — the current choice — MEDIUM, in the header bar.

*field, declared at [`include/shulib/competition/auton_menu.hpp:152`](../../include/shulib/competition/auton_menu.hpp#L152).*

<a id="menuview-info"></a>

### `MenuView::info`

```cpp
MenuLine info{}
```

The choice's kind, schedule and start mode (or what NONE means), SMALL.

*field, declared at [`include/shulib/competition/auton_menu.hpp:154`](../../include/shulib/competition/auton_menu.hpp#L154).*

<a id="menuview-status"></a>

### `MenuView::status`

```cpp
MenuLine status{}
```

The hint, or LOCKED, and the page — SMALL.

*field, declared at [`include/shulib/competition/auton_menu.hpp:156`](../../include/shulib/competition/auton_menu.hpp#L156).*

<a id="menuview-buttons"></a>

### `MenuView::buttons`

```cpp
MenuButton buttons[kMenuMaxButtons]{}
```

The buttons, in drawing order: the page's autons, then PREV (if any), NONE, NEXT (if any).

*field, declared at [`include/shulib/competition/auton_menu.hpp:158`](../../include/shulib/competition/auton_menu.hpp#L158).*

<a id="menuview-buttoncount"></a>

### `MenuView::buttonCount`

```cpp
std::uint8_t buttonCount = 0
```

How many entries of `buttons` are real.

*field, declared at [`include/shulib/competition/auton_menu.hpp:160`](../../include/shulib/competition/auton_menu.hpp#L160).*

<a id="menuview-page"></a>

### `MenuView::page`

```cpp
int page = 0
```

The page shown, 0-based.

*field, declared at [`include/shulib/competition/auton_menu.hpp:162`](../../include/shulib/competition/auton_menu.hpp#L162).*

<a id="menuview-pagecount"></a>

### `MenuView::pageCount`

```cpp
int pageCount = 1
```

How many pages there are (at least 1 — the page with only NONE when nothing is registered).

*field, declared at [`include/shulib/competition/auton_menu.hpp:164`](../../include/shulib/competition/auton_menu.hpp#L164).*

<a id="menuview-locked"></a>

### `MenuView::locked`

```cpp
bool locked = false
```

The choice is locked: every button is dead.

*field, declared at [`include/shulib/competition/auton_menu.hpp:166`](../../include/shulib/competition/auton_menu.hpp#L166).*

<a id="enum-class-menutouchaction"></a>

## `enum class MenuTouchAction`

```cpp
enum class MenuTouchAction : std::uint8_t
```

What a touch did.

*enum class, declared at [`include/shulib/competition/auton_menu.hpp:170`](../../include/shulib/competition/auton_menu.hpp#L170).*

<a id="menutouchaction-none"></a>

### `MenuTouchAction::None`

```cpp
None
```

nothing — no new release, a miss, or a greyed button

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:171`](../../include/shulib/competition/auton_menu.hpp#L171).*

<a id="menutouchaction-chose"></a>

### `MenuTouchAction::Chose`

```cpp
Chose
```

the choice changed (or was re-chosen) — MenuTouch::choice holds it

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:172`](../../include/shulib/competition/auton_menu.hpp#L172).*

<a id="menutouchaction-paged"></a>

### `MenuTouchAction::Paged`

```cpp
Paged
```

the page changed

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:173`](../../include/shulib/competition/auton_menu.hpp#L173).*

<a id="menutouchaction-refusedlocked"></a>

### `MenuTouchAction::RefusedLocked`

```cpp
RefusedLocked
```

a live-looking button was hit after the choice locked — nothing changed

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:174`](../../include/shulib/competition/auton_menu.hpp#L174).*

<a id="struct-menutouch"></a>

## `struct MenuTouch`

```cpp
struct MenuTouch
```

The outcome of one touch read.

*struct, declared at [`include/shulib/competition/auton_menu.hpp:178`](../../include/shulib/competition/auton_menu.hpp#L178).*

<a id="menutouch-action"></a>

### `MenuTouch::action`

```cpp
MenuTouchAction action = MenuTouchAction::None
```

What happened.

*field, declared at [`include/shulib/competition/auton_menu.hpp:180`](../../include/shulib/competition/auton_menu.hpp#L180).*

<a id="menutouch-hit"></a>

### `MenuTouch::hit`

```cpp
int hit = -1
```

The button index hit in the DRAWN view (-1 for none) — for the log.

*field, declared at [`include/shulib/competition/auton_menu.hpp:182`](../../include/shulib/competition/auton_menu.hpp#L182).*

<a id="menutouch-newrelease"></a>

### `MenuTouch::newRelease`

```cpp
bool newRelease = false
```

True when this read carried a NEW release (logged even when it hits nothing).

*field, declared at [`include/shulib/competition/auton_menu.hpp:184`](../../include/shulib/competition/auton_menu.hpp#L184).*

<a id="menutouch-choice"></a>

### `MenuTouch::choice`

```cpp
int choice = -1
```

The choice after this touch: a registry index, or -1 for NONE.

*field, declared at [`include/shulib/competition/auton_menu.hpp:186`](../../include/shulib/competition/auton_menu.hpp#L186).*

<a id="class-autonmenu"></a>

## `class AutonMenu`

```cpp
class AutonMenu
```

The selector (file banner). Borrows the registry, which must be COMPLETE before the first view() and outlive the menu. view() and touch() belong to ONE task (the one that paints); choice(), chosenEntry(), lockChoice(), unlock() and locked() may be called from any task.

*class, declared at [`include/shulib/competition/auton_menu.hpp:192`](../../include/shulib/competition/auton_menu.hpp#L192).*

<a id="autonmenu-autonmenu"></a>

### `AutonMenu::AutonMenu`

```cpp
AutonMenu(const AutonRegistry& registry, std::int32_t releaseCountAtStart) noexcept
```

`releaseCountAtStart`: PROS's release_count when the menu first appears, so a tap made before it existed is never replayed as a choice. The choice starts at NONE.

*function, declared at [`include/shulib/competition/auton_menu.hpp:196`](../../include/shulib/competition/auton_menu.hpp#L196).*

<a id="autonmenu-autonmenu-2"></a>

### `AutonMenu::AutonMenu (overload 2)`

```cpp
AutonMenu(const AutonMenu&) = delete
```

Not copyable or movable: the choice is an atomic other tasks read by address.

*function, declared at [`include/shulib/competition/auton_menu.hpp:200`](../../include/shulib/competition/auton_menu.hpp#L200).*

<a id="autonmenu-operator-eq"></a>

### `AutonMenu::operator=`

```cpp
AutonMenu& operator=(const AutonMenu&) = delete
```

Not copyable or movable: the choice is an atomic other tasks read by address. See the copy constructor.

*function, declared at [`include/shulib/competition/auton_menu.hpp:202`](../../include/shulib/competition/auton_menu.hpp#L202).*

<a id="autonmenu-autonmenu-3"></a>

### `AutonMenu::AutonMenu (overload 3)`

```cpp
AutonMenu(AutonMenu&&) = delete
```

Not copyable or movable: the choice is an atomic other tasks read by address. See the copy constructor. See the copy constructor.

*function, declared at [`include/shulib/competition/auton_menu.hpp:204`](../../include/shulib/competition/auton_menu.hpp#L204).*

<a id="autonmenu-operator-eq-2"></a>

### `AutonMenu::operator= (overload 2)`

```cpp
AutonMenu& operator=(AutonMenu&&) = delete
```

Not copyable or movable: the choice is an atomic other tasks read by address. See the copy constructor. See the copy constructor. See the copy constructor.

*function, declared at [`include/shulib/competition/auton_menu.hpp:206`](../../include/shulib/competition/auton_menu.hpp#L206).*

<a id="autonmenu-destructor-autonmenu"></a>

### `AutonMenu::~AutonMenu`

```cpp
~AutonMenu() = default
```

Not copyable or movable: the choice is an atomic other tasks read by address. See the copy constructor. See the copy constructor. See the copy constructor. Owns nothing that needs releasing.

*function, declared at [`include/shulib/competition/auton_menu.hpp:208`](../../include/shulib/competition/auton_menu.hpp#L208).*

<a id="autonmenu-view"></a>

### `AutonMenu::view`

```cpp
[[nodiscard]] MenuView view(double nowSeconds)
```

The view to paint at `nowSeconds`. Restarts the arming clock whenever the layout (page and buttons) differs from the last view this returned, so a new layout's buttons come up greyed on the very first frame they appear.

*function, declared at [`include/shulib/competition/auton_menu.hpp:213`](../../include/shulib/competition/auton_menu.hpp#L213).*

<a id="autonmenu-touch"></a>

### `AutonMenu::touch`

```cpp
[[nodiscard]] MenuTouch touch(const ui::TouchSample& s, const MenuView& drawn)
```

One touch read against the view that is ON THE SCREEN (`drawn`). A new release on an armed button chooses its auton (or NONE) or flips the page; once locked, nothing changes.

*function, declared at [`include/shulib/competition/auton_menu.hpp:221`](../../include/shulib/competition/auton_menu.hpp#L221).*

<a id="autonmenu-choice"></a>

### `AutonMenu::choice`

```cpp
[[nodiscard]] int choice() const noexcept
```

The current choice: a registry index, or -1 for NONE (the default). Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:262`](../../include/shulib/competition/auton_menu.hpp#L262).*

<a id="autonmenu-chosenentry"></a>

### `AutonMenu::chosenEntry`

```cpp
[[nodiscard]] const AutonEntry* chosenEntry() const noexcept
```

The chosen entry, or nullptr for NONE. Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:265`](../../include/shulib/competition/auton_menu.hpp#L265).*

<a id="autonmenu-lockchoice"></a>

### `AutonMenu::lockChoice`

```cpp
int lockChoice() noexcept
```

LOCK the choice and return it, in ONE atomic operation — the auton that runs is the choice at this instant, and no touch after it changes the choice (file banner). Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:272`](../../include/shulib/competition/auton_menu.hpp#L272).*

<a id="autonmenu-unlock"></a>

### `AutonMenu::unlock`

```cpp
void unlock() noexcept
```

Unlock: touches may change the choice again. For the program to call only after the auton it locked for has FINISHED (a second autonomous() on the same boot then takes the choice current at ITS start). Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:277`](../../include/shulib/competition/auton_menu.hpp#L277).*

<a id="autonmenu-locked"></a>

### `AutonMenu::locked`

```cpp
[[nodiscard]] bool locked() const noexcept
```

True while locked. Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:280`](../../include/shulib/competition/auton_menu.hpp#L280).*

<a id="autonmenu-page"></a>

### `AutonMenu::page`

```cpp
[[nodiscard]] int page() const noexcept
```

The page shown, 0-based.

*function, declared at [`include/shulib/competition/auton_menu.hpp:283`](../../include/shulib/competition/auton_menu.hpp#L283).*

<a id="autonmenu-pagecount"></a>

### `AutonMenu::pageCount`

```cpp
[[nodiscard]] int pageCount() const noexcept
```

How many pages the registry needs (at least 1).

*function, declared at [`include/shulib/competition/auton_menu.hpp:286`](../../include/shulib/competition/auton_menu.hpp#L286).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 36 lines</summary>

```text

 competition::AutonMenu — the brain-screen auton selector as a PURE MODEL (chunk COMP-a,
 2026-09-26; the COMP brief's ruling C4).

 ── What it decides, and why it is a model ──────────────────────────────────────────
 The selector is the last thing a person touches before a match, usually in a hurry, often with
 gloves, on a resistive screen whose release point wanders. Everything that decides WHAT it says
 and WHICH touches mean something lives here, where the host suite can walk every reachable view
 — the calibrate panel's lesson (calibration/calibrate_panel.hpp), built on the same generic
 pieces (ui/touch_panel.hpp). The program only RENDERS a MenuView and hands in touch samples.

 The rules, each pinned by a test:
   * NONE is ALWAYS on the screen, in the SAME place, and it is the DEFAULT: a program that boots
     and is never touched runs no auton. (A robot that drives off the line because nobody chose
     is the failure; a robot that sits still is the safe one.)
   * One button per registered auton, labelled "<name> <KIND>", PAGED when more than
     kMenuEntriesPerPage are registered; paging hides none (every entry is on exactly one page).
   * No new button appears under the one just pressed: entries live in the middle of the screen,
     NONE / PREV / NEXT in the bottom row, and a page flip either keeps a button where it was (the
     same id) or leaves that place empty — so a reflexive double tap cannot answer a question the
     first tap asked.
   * After every layout change the new buttons are greyed and dead for ui::kArmingDelaySeconds,
     except NONE, which is never delayed (choosing no auton is always safe).
   * The CHOICE is one atomic value, readable from any task; lockChoice() freezes it and returns
     it in the same atomic operation, so a touch racing the auton's start can never change the
     auton that runs — and, once locked, no touch changes it at all (every button is dead).
   * Every string it draws passes ui::screenText(): printable ASCII, no '%', within its budget —
     literals by static_assert, names because the registry refused anything else.

 ── What it does NOT decide ─────────────────────────────────────────────────────────
 Which task polls the screen, when to lock and unlock (the program: lock when the auton task
 starts the auton, unlock — if it wants a second autonomous() to take a new choice — only after
 that task has finished), pixels, fonts, RGB, and repainting only what changed. NOTHING HERE HAS
 RUN ON A ROBOT.

 PROS-free, allocation-free, header-only.
```

</details>
