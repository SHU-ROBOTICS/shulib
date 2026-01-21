# Competition Checklist

**Pre-match checklist to ensure everything works**

---

## Night Before Competition

### Hardware
- [ ] All batteries fully charged (at least 3)
- [ ] Battery charger packed
- [ ] Spare parts kit packed
- [ ] Tools packed
- [ ] Controller charged

### Software
- [ ] Code compiles without errors
- [ ] Correct robot selected in `config.hpp`
- [ ] Autonomous routines tested
- [ ] Driver controls tested
- [ ] Code pushed to repository

### Documentation
- [ ] Quick reference card printed
- [ ] Control scheme printed
- [ ] Pit setup checklist ready

---

## Arriving at Venue

### Pit Setup
- [ ] Robot unpacked and inspected
- [ ] Controller paired
- [ ] Battery installed and charged
- [ ] Run quick functionality test
- [ ] Fix any transport damage

### Inspection
- [ ] Robot passes size (18" × 18" × 18")
- [ ] Robot passes weight
- [ ] All components legal
- [ ] Robot number visible

---

## Before Each Match

### 5 Minutes Before

- [ ] Check match schedule - confirm your match
- [ ] Identify alliance partners
- [ ] Discuss autonomous strategy with alliance
- [ ] Confirm starting positions

### At Queue

- [ ] Correct autonomous selected in code
- [ ] Fresh battery installed
- [ ] Controller powered on and connected
- [ ] Pneumatics pumped (if applicable)
- [ ] Mechanisms in starting position

### On Field Setup

- [ ] Robot placed in correct starting position
- [ ] Robot aligned properly
- [ ] Mechanisms in legal starting config
- [ ] Controller in hand
- [ ] Clear of field elements

---

## Pre-Match Verification (30 seconds)

### Quick Systems Check

```
□ Brain screen on?
□ Motors responding?
□ Controller connected?
```

### Starting Position Check

```
□ Correct tile?
□ Correct orientation?
□ Behind autonomous line?
□ Touching starting elements (if required)?
```

### Mental Checklist

```
□ Which autonomous is running?
□ What's the alliance strategy?
□ Where should I focus during driver?
```

---

## During Match

### Autonomous (15 sec)
- Watch for issues
- Note what worked/failed
- Don't touch anything!

### Driver Control (1:45)
- Execute strategy
- Communicate with alliance
- Watch the clock!
- **REMEMBER TO PARK** (final 15 seconds)

### Endgame
- [ ] 20 sec left → Start thinking about parking
- [ ] 15 sec left → Both robots moving to park
- [ ] 10 sec left → First robot in parking zone
- [ ] 5 sec left → Both robots parked
- [ ] 0 sec → DON'T MOVE!

---

## After Each Match

### Immediate
- [ ] Retrieve robot carefully
- [ ] Check for damage
- [ ] Note any issues

### In Pits
- [ ] Inspect robot thoroughly
- [ ] Fix any problems
- [ ] Swap battery if needed
- [ ] Review match (what worked, what didn't)

### If Issues Found
- [ ] Diagnose problem
- [ ] Fix or work around
- [ ] Test fix before next match

---

## Code Changes at Competition

### Before Changing Anything
- [ ] Current code works (mostly)
- [ ] Change is necessary and small
- [ ] You understand the change

### Making the Change
- [ ] Change ONE thing at a time
- [ ] Build and verify compiles
- [ ] Test on robot before match
- [ ] Commit to git

### If Things Go Wrong
```bash
# Revert to last working code
git checkout HEAD -- <file>

# Or discard all changes
git checkout .
```

---

## Emergency Procedures

### Robot Won't Turn On
1. Check battery connection
2. Try different battery
3. Check brain power button
4. Restart brain (hold button)

### Code Won't Upload
1. Check USB connection
2. Restart brain
3. Restart computer
4. Try different USB port/cable

### Motors Don't Work
1. Check port connections
2. Check motor cables
3. Verify port numbers in code
4. Test motor individually

### Controller Won't Connect
1. Check controller is on
2. Re-pair controller (tether required)
3. Try backup controller

### Autonomous Doesn't Run
1. Verify competition mode
2. Check autonomous selection
3. Verify code uploaded

---

## Printable Match Card

```
╔═══════════════════════════════════════════════════╗
║             MATCH CHECKLIST                       ║
╠═══════════════════════════════════════════════════╣
║                                                   ║
║  □ Correct autonomous selected                    ║
║  □ Fresh battery                                  ║
║  □ Controller connected                           ║
║  □ Pneumatics pumped                              ║
║  □ Starting position correct                      ║
║  □ Mechanisms in start config                     ║
║                                                   ║
╠═══════════════════════════════════════════════════╣
║                                                   ║
║  AUTON: ____________________                      ║
║                                                   ║
║  STRATEGY: _____________________                  ║
║                                                   ║
║  PARK AT: 15 seconds left!                        ║
║                                                   ║
╚═══════════════════════════════════════════════════╝
```

---

## Match Strategy Template

Fill out before each match:

```
Match #: _______

Alliance Color: RED / BLUE

Starting Position: LEFT / RIGHT

Autonomous: _______________________

Alliance Partner Strategy:
________________________________
________________________________

My Focus:
________________________________
________________________________

Parking Plan:
□ I park first  □ Partner parks first
```

---

## Packing List

### Robot & Electronics
- [ ] Robot
- [ ] V5 Brain (on robot)
- [ ] Controller (×2 if available)
- [ ] Batteries (×3 minimum)
- [ ] Battery charger
- [ ] USB cable

### Tools
- [ ] Hex keys (complete set)
- [ ] Screwdrivers
- [ ] Pliers
- [ ] Zip ties
- [ ] Electrical tape
- [ ] Rubber bands

### Spare Parts
- [ ] Spare motors
- [ ] Spare wheels
- [ ] Spare shafts
- [ ] Spare gears
- [ ] Screws (assorted)
- [ ] Nuts (assorted)
- [ ] Standoffs

### Documentation
- [ ] Quick reference card
- [ ] Control scheme
- [ ] This checklist
- [ ] Notebook for notes

### Other
- [ ] Laptop (charged)
- [ ] Laptop charger
- [ ] Extension cord
- [ ] Team banner/signage
- [ ] Water/snacks

---

*For quick reference info, see [QUICK_REFERENCE.md](./QUICK_REFERENCE.md)*
*For controls, see [pushback_2026/CONTROLS.md](../seasons/pushback_2026/CONTROLS.md)*