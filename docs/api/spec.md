<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/spec.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `spec.hpp`

mechanism/spec.hpp — ONE MECHANISM AS DATA, and THE BUILD CHECK.

This header declares **18** types (195 members), **78** free functions, and **7** constants.

Extracted from [`include/shulib/mechanism/spec.hpp`](../../include/shulib/mechanism/spec.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxMechanisms`](#kmaxmechanisms) — *constant*
- [`kMaxMotorRoles`](#kmaxmotorroles) — *constant*
- [`kMaxLineRoles`](#kmaxlineroles) — *constant*
- [`kMaxPresets`](#kmaxpresets) — *constant*
- [`kMaxPneumaticPositions`](#kmaxpneumaticpositions) — *constant*
- [`kMaxTablePoints`](#kmaxtablepoints) — *constant*
- [`kNameBytes`](#knamebytes) — *constant*
- [`enum class Kind`](#enum-class-kind)
  - [`Power`](#kind-power)
  - [`Position`](#kind-position)
  - [`Speed`](#kind-speed)
  - [`Pneumatic`](#kind-pneumatic)
- [`kindWord`](#kindword) — *free function*
- [`enum class SensorKind`](#enum-class-sensorkind)
  - [`None`](#sensorkind-none)
  - [`Rotation`](#sensorkind-rotation)
  - [`Switch`](#sensorkind-switch)
  - [`Distance`](#sensorkind-distance)
  - [`Optical`](#sensorkind-optical)
- [`struct MotorRole`](#struct-motorrole)
  - [`reversed`](#motorrole-reversed)
  - [`source`](#motorrole-source)
  - [`file`](#motorrole-file)
  - [`line`](#motorrole-line)
- [`struct SensorRole`](#struct-sensorrole)
  - [`kind`](#sensorrole-kind)
  - [`reversed`](#sensorrole-reversed)
  - [`pressedLevel`](#sensorrole-pressedlevel)
  - [`seesWithinIn`](#sensorrole-seeswithinin)
  - [`seesProximity`](#sensorrole-seesproximity)
  - [`source`](#sensorrole-source)
  - [`file`](#sensorrole-file)
  - [`line`](#sensorrole-line)
- [`enum class Units`](#enum-class-units)
  - [`Degrees`](#units-degrees)
  - [`Inches`](#units-inches)
- [`enum class Geometry`](#enum-class-geometry)
  - [`Rotary`](#geometry-rotary)
  - [`Spool`](#geometry-spool)
  - [`Arm`](#geometry-arm)
  - [`Table`](#geometry-table)
- [`enum class Gravity`](#enum-class-gravity)
  - [`None`](#gravity-none)
  - [`Cosine`](#gravity-cosine)
  - [`Constant`](#gravity-constant)
- [`enum class HomeMethod`](#enum-class-homemethod)
  - [`None`](#homemethod-none)
  - [`RestsAtStop`](#homemethod-restsatstop)
  - [`Switch`](#homemethod-switch)
  - [`Stall`](#homemethod-stall)
  - [`Absolute`](#homemethod-absolute)
- [`struct Preset`](#struct-preset)
  - [`name`](#preset-name)
  - [`value`](#preset-value)
- [`struct StallSpec`](#struct-stallspec)
  - [`amps`](#stallspec-amps)
  - [`speedRadS`](#stallspec-speedrads)
  - [`time`](#stallspec-time)
  - [`stated`](#stallspec-stated)
  - [`written`](#stallspec-written)
- [`struct PowerSpec`](#struct-powerspec)
  - [`maxVolts`](#powerspec-maxvolts)
  - [`stall`](#powerspec-stall)
- [`struct PositionSpec`](#struct-positionspec)
  - [`units`](#positionspec-units)
  - [`geometry`](#positionspec-geometry)
  - [`gearRatio`](#positionspec-gearratio)
  - [`spoolDiameterIn`](#positionspec-spooldiameterin)
  - [`armLengthIn`](#positionspec-armlengthin)
  - [`pivotHeightIn`](#positionspec-pivotheightin)
  - [`armStages`](#positionspec-armstages)
  - [`table`](#positionspec-table)
  - [`tablePoints`](#positionspec-tablepoints)
  - [`minPosition`](#positionspec-minposition)
  - [`maxPosition`](#positionspec-maxposition)
  - [`maxVelocity`](#positionspec-maxvelocity)
  - [`maxAcceleration`](#positionspec-maxacceleration)
  - [`tolerance`](#positionspec-tolerance)
  - [`settleTime`](#positionspec-settletime)
  - [`kP`](#positionspec-kp)
  - [`kI`](#positionspec-ki)
  - [`kD`](#positionspec-kd)
  - [`kS`](#positionspec-ks)
  - [`kV`](#positionspec-kv)
  - [`kA`](#positionspec-ka)
  - [`kG`](#positionspec-kg)
  - [`gravity`](#positionspec-gravity)
  - [`horizontalAt`](#positionspec-horizontalat)
  - [`home`](#positionspec-home)
  - [`homePosition`](#positionspec-homeposition)
  - [`homeVolts`](#positionspec-homevolts)
  - [`homeStall`](#positionspec-homestall)
  - [`stall`](#positionspec-stall)
  - [`unhomedVolts`](#positionspec-unhomedvolts)
  - [`absoluteSensorAtZeroDeg`](#positionspec-absolutesensoratzerodeg)
  - [`presets`](#positionspec-presets)
  - [`presetCount`](#positionspec-presetcount)
- [`struct SpeedSpec`](#struct-speedspec)
  - [`gearRatio`](#speedspec-gearratio)
  - [`maxRpm`](#speedspec-maxrpm)
  - [`kS`](#speedspec-ks)
  - [`kV`](#speedspec-kv)
  - [`kA`](#speedspec-ka)
  - [`kP`](#speedspec-kp)
  - [`kI`](#speedspec-ki)
  - [`atSpeedBandRpm`](#speedspec-atspeedbandrpm)
  - [`atSpeedTime`](#speedspec-atspeedtime)
  - [`presets`](#speedspec-presets)
  - [`presetCount`](#speedspec-presetcount)
  - [`stall`](#speedspec-stall)
- [`enum class Confirm`](#enum-class-confirm)
  - [`None`](#confirm-none)
  - [`Sees`](#confirm-sees)
  - [`StopsSeeing`](#confirm-stopsseeing)
- [`struct PneumaticPositionSpec`](#struct-pneumaticpositionspec)
  - [`name`](#pneumaticpositionspec-name)
  - [`levels`](#pneumaticpositionspec-levels)
  - [`confirm`](#pneumaticpositionspec-confirm)
  - [`levelCount`](#pneumaticpositionspec-levelcount)
- [`struct PneumaticSpec`](#struct-pneumaticspec)
  - [`positions`](#pneumaticspec-positions)
  - [`positionCount`](#pneumaticspec-positioncount)
  - [`start`](#pneumaticspec-start)
  - [`safe`](#pneumaticspec-safe)
  - [`actuationTime`](#pneumaticspec-actuationtime)
  - [`confirmWindow`](#pneumaticspec-confirmwindow)
  - [`levelsSource`](#pneumaticspec-levelssource)
  - [`levelsFile`](#pneumaticspec-levelsfile)
  - [`levelsLine`](#pneumaticspec-levelsline)
  - [`startNamed`](#pneumaticspec-startnamed)
  - [`safeNamed`](#pneumaticspec-safenamed)
- [`struct MechanismSpec`](#struct-mechanismspec)
  - [`name`](#mechanismspec-name)
  - [`kind`](#mechanismspec-kind)
  - [`motors`](#mechanismspec-motors)
  - [`motorCount`](#mechanismspec-motorcount)
  - [`lineCount`](#mechanismspec-linecount)
  - [`angleSensor`](#mechanismspec-anglesensor)
  - [`homeSwitch`](#mechanismspec-homeswitch)
  - [`pieceSensor`](#mechanismspec-piecesensor)
  - [`cartridge`](#mechanismspec-cartridge)
  - [`safeMode`](#mechanismspec-safemode)
  - [`power`](#mechanismspec-power)
  - [`position`](#mechanismspec-position)
  - [`speed`](#mechanismspec-speed)
  - [`pneumatic`](#mechanismspec-pneumatic)
  - [`declaredFile`](#mechanismspec-declaredfile)
  - [`declaredLine`](#mechanismspec-declaredline)
  - [`presetNameTooLong`](#mechanismspec-presetnametoolong)
- [`nameOf`](#nameof) — *free function*
- [`angleDegToPosition`](#angledegtoposition) — *free function*
- [`positionToAngleDeg`](#positiontoangledeg) — *free function*
- [`enum class Refusal`](#enum-class-refusal)
  - [`None`](#refusal-none)
  - [`AMechanismHasNoName`](#refusal-amechanismhasnoname)
  - [`AMechanismNameDoesNotFit23Characters`](#refusal-amechanismnamedoesnotfit23characters)
  - [`AMechanismSKindIsUNSET`](#refusal-amechanismskindisunset)
  - [`AMotorMechanismNeeds14MotorRoles`](#refusal-amotormechanismneeds14motorroles)
  - [`AMotorMechanismHas3WireLineRolesMakeTwoMechanisms`](#refusal-amotormechanismhas3wirelinerolesmaketwomechanisms)
  - [`AMotorMechanismSCartridgeIsUNSET`](#refusal-amotormechanismscartridgeisunset)
  - [`AMotorMechanismSSafeBrakeModeIsUNSET`](#refusal-amotormechanismssafebrakemodeisunset)
  - [`APneumaticMechanismHasMotorRoles`](#refusal-apneumaticmechanismhasmotorroles)
  - [`APneumaticMechanismNeeds12LineRoles`](#refusal-apneumaticmechanismneeds12lineroles)
  - [`TheAngleSensorRoleIsNotARotationSensor`](#refusal-theanglesensorroleisnotarotationsensor)
  - [`AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed`](#refusal-ananglesensorroleonamechanismthatisnotpositionorspeed)
  - [`TheHomeSwitchRoleIsNotALimitSwitch`](#refusal-thehomeswitchroleisnotalimitswitch)
  - [`AHomeSwitchRoleOnAMechanismThatIsNotPosition`](#refusal-ahomeswitchroleonamechanismthatisnotposition)
  - [`APieceSensorCannotBeARotationSensor`](#refusal-apiecesensorcannotbearotationsensor)
  - [`ADistancePieceSensorNeedsItsSeesWithinDistance0`](#refusal-adistancepiecesensorneedsitsseeswithindistance0)
  - [`AnOpticalPieceSensorNeedsItsProximityThresholdIn01`](#refusal-anopticalpiecesensorneedsitsproximitythresholdin01)
  - [`PowerMaxVoltsIsNotIn012`](#refusal-powermaxvoltsisnotin012)
  - [`PositionUnitsUNSET`](#refusal-positionunitsunset)
  - [`PositionGearRatioMustBe0`](#refusal-positiongearratiomustbe0)
  - [`PositionARotaryMechanismIsInDegrees`](#refusal-positionarotarymechanismisindegrees)
  - [`PositionALiftIsInInches`](#refusal-positionaliftisininches)
  - [`PositionSpoolDiameterMustBe0`](#refusal-positionspooldiametermustbe0)
  - [`PositionArmLengthMustBe0`](#refusal-positionarmlengthmustbe0)
  - [`PositionAnArmNeeds13Stages`](#refusal-positionanarmneeds13stages)
  - [`PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul`](#refusal-positionanarmstravellimitsreachpastitslinkagetheheightformul)
  - [`PositionALookupTableNeeds28Points`](#refusal-positionalookuptableneeds28points)
  - [`PositionTheLookupTableIsNotStrictlyIncreasing`](#refusal-positionthelookuptableisnotstrictlyincreasing)
  - [`PositionTravelLimitsMissingOrMinMax`](#refusal-positiontravellimitsmissingorminmax)
  - [`PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0`](#refusal-positiontheprofileneedsmaxvelocityandmaxacceleration0)
  - [`PositionToleranceMustBe0`](#refusal-positiontolerancemustbe0)
  - [`PositionGravityUNSET`](#refusal-positiongravityunset)
  - [`PositionHomingMethodUNSET`](#refusal-positionhomingmethodunset)
  - [`PositionHomingBySwitchNeedsAHomeSwitchRole`](#refusal-positionhomingbyswitchneedsahomeswitchrole)
  - [`PositionAbsoluteHomingNeedsAnAngleSensorRotationRole`](#refusal-positionabsolutehomingneedsananglesensorrotationrole)
  - [`PositionHomingNeedsHomeVolts`](#refusal-positionhomingneedshomevolts)
  - [`PositionStallHomingNeedsItsStallSignature`](#refusal-positionstallhomingneedsitsstallsignature)
  - [`PositionTheHomePositionIsOutsideTheTravelLimits`](#refusal-positionthehomepositionisoutsidethetravellimits)
  - [`PositionMorePresetsThanFit`](#refusal-positionmorepresetsthanfit)
  - [`PositionAPresetHasNoName`](#refusal-positionapresethasnoname)
  - [`PositionAPresetIsOutsideTheTravelLimits`](#refusal-positionapresetisoutsidethetravellimits)
  - [`SpeedGearRatioAndMaxRpmMustBe0`](#refusal-speedgearratioandmaxrpmmustbe0)
  - [`SpeedTheAtSpeedBandMustBe0`](#refusal-speedtheatspeedbandmustbe0)
  - [`SpeedMorePresetsThanFit`](#refusal-speedmorepresetsthanfit)
  - [`SpeedAPresetHasNoName`](#refusal-speedapresethasnoname)
  - [`SpeedAPresetIsPastMaxRpm`](#refusal-speedapresetispastmaxrpm)
  - [`Pneumatic2Or3Positions`](#refusal-pneumatic2or3positions)
  - [`PneumaticStartSafeIsNotOneOfItsPositions`](#refusal-pneumaticstartsafeisnotoneofitspositions)
  - [`PneumaticActuationTimeMustBe0`](#refusal-pneumaticactuationtimemustbe0)
  - [`PneumaticAPositionHasNoName`](#refusal-pneumaticapositionhasnoname)
  - [`PneumaticTwoPositionsHaveTheSameLineLevels`](#refusal-pneumatictwopositionshavethesamelinelevels)
  - [`PneumaticItsStartNamesNoneOfItsPositions`](#refusal-pneumaticitsstartnamesnoneofitspositions)
  - [`PneumaticItsSafeNamesNoneOfItsPositions`](#refusal-pneumaticitssafenamesnoneofitspositions)
  - [`APresetOrPositionNameDoesNotFit15Characters`](#refusal-apresetorpositionnamedoesnotfit15characters)
  - [`TwoPresetsOrPositionsShareAName`](#refusal-twopresetsorpositionsshareaname)
  - [`AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360`](#refusal-absolutehomingthesensorsreadingatzeromustbein0360)
  - [`PositionTheStallSignatureIsUNSET`](#refusal-positionthestallsignatureisunset)
  - [`PositionHomingPushesAwayFromItsHomeEnd`](#refusal-positionhomingpushesawayfromitshomeend)
  - [`SpeedTheStallSignatureIsUNSET`](#refusal-speedthestallsignatureisunset)
  - [`AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit`](#refusal-astallsignaturesampsmustbebelowthemotors25acurrentlimit)
  - [`AStallSignatureNeedsBelowMotorRpmAbove0`](#refusal-astallsignatureneedsbelowmotorrpmabove0)
  - [`AStallSignatureNeedsLastingAbove0`](#refusal-astallsignatureneedslastingabove0)
  - [`AStallSignatureNeedsAmpsAbove0`](#refusal-astallsignatureneedsampsabove0)
  - [`PositionGeometryUNSET`](#refusal-positiongeometryunset)
  - [`PositionTheSettleTimeIsNegative`](#refusal-positionthesettletimeisnegative)
  - [`SpeedTheAtSpeedLastingIsNegative`](#refusal-speedtheatspeedlastingisnegative)
  - [`PneumaticTheConfirmWindowIsNegative`](#refusal-pneumatictheconfirmwindowisnegative)
  - [`PositionTheHomingPushIsPast12Volts`](#refusal-positionthehomingpushispast12volts)
  - [`PositionTheGravityHoldIsPast12Volts`](#refusal-positionthegravityholdispast12volts)
  - [`PositionUnhomedVoltsIsNotIn0To12`](#refusal-positionunhomedvoltsisnotin0to12)
  - [`PositionAbsoluteHomingTravelSpansAWholeSensorTurn`](#refusal-positionabsolutehomingtravelspansawholesensorturn)
- [`refusalText`](#refusaltext) — *free function*
- [`a_mechanism_has_no_name`](#a_mechanism_has_no_name) — *free function*
- [`a_mechanism_name_does_not_fit_twenty_three_characters`](#a_mechanism_name_does_not_fit_twenty_three_characters) — *free function*
- [`a_mechanisms_kind_is_unset`](#a_mechanisms_kind_is_unset) — *free function*
- [`a_motor_mechanism_needs_one_to_four_motor_roles`](#a_motor_mechanism_needs_one_to_four_motor_roles) — *free function*
- [`a_motor_mechanism_has_three_wire_line_roles_make_two_mechanisms`](#a_motor_mechanism_has_three_wire_line_roles_make_two_mechanisms) — *free function*
- [`a_motor_mechanisms_cartridge_is_unset`](#a_motor_mechanisms_cartridge_is_unset) — *free function*
- [`a_motor_mechanisms_safe_brake_mode_is_unset`](#a_motor_mechanisms_safe_brake_mode_is_unset) — *free function*
- [`a_pneumatic_mechanism_has_motor_roles`](#a_pneumatic_mechanism_has_motor_roles) — *free function*
- [`a_pneumatic_mechanism_needs_one_or_two_line_roles`](#a_pneumatic_mechanism_needs_one_or_two_line_roles) — *free function*
- [`the_angle_sensor_role_is_not_a_rotation_sensor`](#the_angle_sensor_role_is_not_a_rotation_sensor) — *free function*
- [`an_angle_sensor_role_on_a_mechanism_that_is_not_position_or_speed`](#an_angle_sensor_role_on_a_mechanism_that_is_not_position_or_speed) — *free function*
- [`the_home_switch_role_is_not_a_limit_switch`](#the_home_switch_role_is_not_a_limit_switch) — *free function*
- [`a_home_switch_role_on_a_mechanism_that_is_not_position`](#a_home_switch_role_on_a_mechanism_that_is_not_position) — *free function*
- [`a_piece_sensor_cannot_be_a_rotation_sensor`](#a_piece_sensor_cannot_be_a_rotation_sensor) — *free function*
- [`a_distance_piece_sensor_needs_its_sees_within_distance_more_than_zero`](#a_distance_piece_sensor_needs_its_sees_within_distance_more_than_zero) — *free function*
- [`an_optical_piece_sensor_needs_its_proximity_threshold_in_above_0_and_at_most_1`](#an_optical_piece_sensor_needs_its_proximity_threshold_in_above_0_and_at_most_1) — *free function*
- [`power_maxvolts_is_not_in_above_0_and_at_most_12`](#power_maxvolts_is_not_in_above_0_and_at_most_12) — *free function*
- [`position_units_unset`](#position_units_unset) — *free function*
- [`position_gearratio_must_be_more_than_zero`](#position_gearratio_must_be_more_than_zero) — *free function*
- [`position_a_rotary_mechanism_is_in_degrees`](#position_a_rotary_mechanism_is_in_degrees) — *free function*
- [`position_a_lift_is_in_inches`](#position_a_lift_is_in_inches) — *free function*
- [`position_spool_diameter_must_be_more_than_zero`](#position_spool_diameter_must_be_more_than_zero) — *free function*
- [`position_arm_length_must_be_more_than_zero`](#position_arm_length_must_be_more_than_zero) — *free function*
- [`position_an_arm_needs_one_to_three_stages`](#position_an_arm_needs_one_to_three_stages) — *free function*
- [`position_an_arms_travel_limits_reach_past_its_linkage_the_height_formula_has_no_inverse_there`](#position_an_arms_travel_limits_reach_past_its_linkage_the_height_formula_has_no_inverse_there) — *free function*
- [`position_a_lookup_table_needs_two_to_eight_points`](#position_a_lookup_table_needs_two_to_eight_points) — *free function*
- [`position_the_lookup_table_is_not_strictly_increasing`](#position_the_lookup_table_is_not_strictly_increasing) — *free function*
- [`position_travel_limits_missing_or_min_at_least_max`](#position_travel_limits_missing_or_min_at_least_max) — *free function*
- [`position_the_profile_needs_maxvelocity_and_maxacceleration_more_than_zero`](#position_the_profile_needs_maxvelocity_and_maxacceleration_more_than_zero) — *free function*
- [`position_tolerance_must_be_more_than_zero`](#position_tolerance_must_be_more_than_zero) — *free function*
- [`position_gravity_unset`](#position_gravity_unset) — *free function*
- [`position_homing_method_unset`](#position_homing_method_unset) — *free function*
- [`position_homing_by_switch_needs_a_homeswitch_role`](#position_homing_by_switch_needs_a_homeswitch_role) — *free function*
- [`position_absolute_homing_needs_an_anglesensor_rotation_role`](#position_absolute_homing_needs_an_anglesensor_rotation_role) — *free function*
- [`position_homing_needs_homevolts`](#position_homing_needs_homevolts) — *free function*
- [`position_stall_homing_needs_its_stall_signature`](#position_stall_homing_needs_its_stall_signature) — *free function*
- [`position_the_home_position_is_outside_the_travel_limits`](#position_the_home_position_is_outside_the_travel_limits) — *free function*
- [`position_more_presets_than_fit`](#position_more_presets_than_fit) — *free function*
- [`position_a_preset_has_no_name`](#position_a_preset_has_no_name) — *free function*
- [`position_a_preset_is_outside_the_travel_limits`](#position_a_preset_is_outside_the_travel_limits) — *free function*
- [`speed_gearratio_and_its_free_speed_must_be_more_than_zero`](#speed_gearratio_and_its_free_speed_must_be_more_than_zero) — *free function*
- [`speed_the_at_speed_band_must_be_more_than_zero`](#speed_the_at_speed_band_must_be_more_than_zero) — *free function*
- [`speed_more_presets_than_fit`](#speed_more_presets_than_fit) — *free function*
- [`speed_a_preset_has_no_name`](#speed_a_preset_has_no_name) — *free function*
- [`speed_a_preset_is_past_its_free_speed`](#speed_a_preset_is_past_its_free_speed) — *free function*
- [`pneumatic_two_or_three_positions`](#pneumatic_two_or_three_positions) — *free function*
- [`pneumatic_start_or_safe_is_not_one_of_its_positions`](#pneumatic_start_or_safe_is_not_one_of_its_positions) — *free function*
- [`pneumatic_actuationtime_must_be_more_than_zero`](#pneumatic_actuationtime_must_be_more_than_zero) — *free function*
- [`pneumatic_a_position_has_no_name`](#pneumatic_a_position_has_no_name) — *free function*
- [`pneumatic_two_positions_have_the_same_line_levels`](#pneumatic_two_positions_have_the_same_line_levels) — *free function*
- [`pneumatic_its_start_names_none_of_its_positions`](#pneumatic_its_start_names_none_of_its_positions) — *free function*
- [`pneumatic_its_safe_names_none_of_its_positions`](#pneumatic_its_safe_names_none_of_its_positions) — *free function*
- [`a_preset_or_position_name_does_not_fit_fifteen_characters`](#a_preset_or_position_name_does_not_fit_fifteen_characters) — *free function*
- [`two_presets_or_positions_share_a_name`](#two_presets_or_positions_share_a_name) — *free function*
- [`absolute_homing_the_sensors_reading_at_zero_must_be_in_from_0_up_to_360`](#absolute_homing_the_sensors_reading_at_zero_must_be_in_from_0_up_to_360) — *free function*
- [`position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current`](#position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current) — *free function*
- [`position_the_homing_push_points_away_from_its_home_end`](#position_the_homing_push_points_away_from_its_home_end) — *free function*
- [`speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current`](#speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current) — *free function*
- [`a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_limit`](#a_stall_signatures_amps_must_be_below_the_motors_2_5_a_current_limit) — *free function*
- [`a_stall_signature_needs_belowMotorRpm_above_0`](#a_stall_signature_needs_belowmotorrpm_above_0) — *free function*
- [`a_stall_signature_needs_lasting_above_0`](#a_stall_signature_needs_lasting_above_0) — *free function*
- [`a_stall_signature_needs_its_amps_above_0`](#a_stall_signature_needs_its_amps_above_0) — *free function*
- [`position_a_lifts_shape_is_unset_give_it_spool_linkage_or_measured`](#position_a_lifts_shape_is_unset_give_it_spool_linkage_or_measured) — *free function*
- [`position_the_settle_time_must_not_be_negative`](#position_the_settle_time_must_not_be_negative) — *free function*
- [`speed_the_at_speed_lasting_must_not_be_negative`](#speed_the_at_speed_lasting_must_not_be_negative) — *free function*
- [`pneumatic_confirmwithin_must_not_be_negative`](#pneumatic_confirmwithin_must_not_be_negative) — *free function*
- [`position_the_homing_push_must_be_within_12_volts_either_way`](#position_the_homing_push_must_be_within_12_volts_either_way) — *free function*
- [`position_the_gravity_hold_must_be_within_12_volts_either_way`](#position_the_gravity_hold_must_be_within_12_volts_either_way) — *free function*
- [`position_unhomedvolts_must_be_from_0_to_12`](#position_unhomedvolts_must_be_from_0_to_12) — *free function*
- [`position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor`](#position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor) — *free function*
- [`refuse`](#refuse) — *free function*
- [`specRefusal`](#specrefusal) — *free function*
- [`specProblem`](#specproblem) — *free function*

<a id="kmaxmechanisms"></a>

## `kMaxMechanisms`

```cpp
inline constexpr std::size_t kMaxMechanisms = config::kMaxMechanisms
```

The most mechanisms one robot may declare — the robot config's capacity (8).

*constant, declared at [`include/shulib/mechanism/spec.hpp:59`](../../include/shulib/mechanism/spec.hpp#L59).*

<a id="kmaxmotorroles"></a>

## `kMaxMotorRoles`

```cpp
inline constexpr std::size_t kMaxMotorRoles = config::kMaxMotorsPerMechanism
```

The most MOTOR roles one mechanism may have — the config's per-mechanism motor capacity (4).

*constant, declared at [`include/shulib/mechanism/spec.hpp:61`](../../include/shulib/mechanism/spec.hpp#L61).*

<a id="kmaxlineroles"></a>

## `kMaxLineRoles`

```cpp
inline constexpr std::size_t kMaxLineRoles = config::kMaxPneumaticsPerMechanism
```

The most 3-wire LINE roles one pneumatic mechanism may have — the config's capacity (2).

*constant, declared at [`include/shulib/mechanism/spec.hpp:63`](../../include/shulib/mechanism/spec.hpp#L63).*

<a id="kmaxpresets"></a>

## `kMaxPresets`

```cpp
inline constexpr std::size_t kMaxPresets = 6
```

The most named presets one position or speed mechanism may have.

*constant, declared at [`include/shulib/mechanism/spec.hpp:65`](../../include/shulib/mechanism/spec.hpp#L65).*

<a id="kmaxpneumaticpositions"></a>

## `kMaxPneumaticPositions`

```cpp
inline constexpr std::size_t kMaxPneumaticPositions = 3
```

The most named positions one pneumatic mechanism may have (two, or three on two lines).

*constant, declared at [`include/shulib/mechanism/spec.hpp:67`](../../include/shulib/mechanism/spec.hpp#L67).*

<a id="kmaxtablepoints"></a>

## `kMaxTablePoints`

```cpp
inline constexpr std::size_t kMaxTablePoints = 8
```

The most points a lift's measured height table may have.

*constant, declared at [`include/shulib/mechanism/spec.hpp:69`](../../include/shulib/mechanism/spec.hpp#L69).*

<a id="knamebytes"></a>

## `kNameBytes`

```cpp
inline constexpr std::size_t kNameBytes = 24
```

A mechanism name's storage, NUL included — config::MechanismConfig::name's width (23 characters).

*constant, declared at [`include/shulib/mechanism/spec.hpp:71`](../../include/shulib/mechanism/spec.hpp#L71).*

<a id="enum-class-kind"></a>

## `enum class Kind`

```cpp
enum class Kind : std::uint8_t
```

The four KINDS the core runs — a declaration type in kinds.hpp per behaviour (an Arm and a Lift are both Position; a Flywheel is Speed). The byte 0 is UNSET.

*enum class, declared at [`include/shulib/mechanism/spec.hpp:75`](../../include/shulib/mechanism/spec.hpp#L75).*

<a id="kind-power"></a>

### `Kind::Power`

```cpp
Power = 1
```

run at a voltage (intakes, rollers, conveyors, robot two's claw pivots)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:76`](../../include/shulib/mechanism/spec.hpp#L76).*

<a id="kind-position"></a>

### `Kind::Position`

```cpp
Position = 2
```

go to a position and hold it (an Arm in degrees, a Lift in inches)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:77`](../../include/shulib/mechanism/spec.hpp#L77).*

<a id="kind-speed"></a>

### `Kind::Speed`

```cpp
Speed = 3
```

spin at an output RPM and say when it is "at speed" (a Flywheel)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:78`](../../include/shulib/mechanism/spec.hpp#L78).*

<a id="kind-pneumatic"></a>

### `Kind::Pneumatic`

```cpp
Pneumatic = 4
```

two or three named positions on 3-wire lines

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:79`](../../include/shulib/mechanism/spec.hpp#L79).*

<a id="kindword"></a>

## `kindWord`

```cpp
[[nodiscard]] constexpr const char* kindWord(Kind k) noexcept
```

A kind as a word for boot lines and refusals: "power", "position", "speed", "pneumatic", or "UNSET" for any other byte.

*free function, declared at [`include/shulib/mechanism/spec.hpp:84`](../../include/shulib/mechanism/spec.hpp#L84).*

<a id="enum-class-sensorkind"></a>

## `enum class SensorKind`

```cpp
enum class SensorKind : std::uint8_t
```

What a SENSOR role is. Its port is the wiring's.

*enum class, declared at [`include/shulib/mechanism/spec.hpp:99`](../../include/shulib/mechanism/spec.hpp#L99).*

<a id="sensorkind-none"></a>

### `SensorKind::None`

```cpp
None = 0
```

no such role on this mechanism

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:100`](../../include/shulib/mechanism/spec.hpp#L100).*

<a id="sensorkind-rotation"></a>

### `SensorKind::Rotation`

```cpp
Rotation = 1
```

a V5 rotation sensor (smart port) — an angle sensor

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:101`](../../include/shulib/mechanism/spec.hpp#L101).*

<a id="sensorkind-switch"></a>

### `SensorKind::Switch`

```cpp
Switch = 2
```

a limit switch / bumper (3-wire line)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:102`](../../include/shulib/mechanism/spec.hpp#L102).*

<a id="sensorkind-distance"></a>

### `SensorKind::Distance`

```cpp
Distance = 3
```

a distance sensor as a piece sensor (smart port)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:103`](../../include/shulib/mechanism/spec.hpp#L103).*

<a id="sensorkind-optical"></a>

### `SensorKind::Optical`

```cpp
Optical = 4
```

an optical sensor as a piece sensor (smart port)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:104`](../../include/shulib/mechanism/spec.hpp#L104).*

<a id="struct-motorrole"></a>

## `struct MotorRole`

```cpp
struct MotorRole
```

One MOTOR role: how the motor is MOUNTED (its direction), never where it is plugged in.

*struct, declared at [`include/shulib/mechanism/spec.hpp:108`](../../include/shulib/mechanism/spec.hpp#L108).*

<a id="motorrole-reversed"></a>

### `MotorRole::reversed`

```cpp
bool reversed = false
```

The motor is mounted so that + turns the mechanism the other way (a mirrored lift motor): the projection makes its port NEGATIVE, and its adapter reverses it, once.

*field, declared at [`include/shulib/mechanism/spec.hpp:111`](../../include/shulib/mechanism/spec.hpp#L111).*

<a id="motorrole-source"></a>

### `MotorRole::source`

```cpp
const char* source = nullptr
```

WHERE this role's direction is stated, for the direction check's "change it HERE" (Q1 (a)): a GENERATED robot file's JSON path (G3a). nullptr in a hand-written file — the check then names `file`:`line`.

*field, declared at [`include/shulib/mechanism/spec.hpp:115`](../../include/shulib/mechanism/spec.hpp#L115).*

<a id="motorrole-file"></a>

### `MotorRole::file`

```cpp
const char* file = nullptr
```

The file the declaration was written in (std::source_location, captured by kinds.hpp at the team's own `.name` line) — the path the compiler opened, so host and brain differ (pin a suffix, never the whole path).

*field, declared at [`include/shulib/mechanism/spec.hpp:119`](../../include/shulib/mechanism/spec.hpp#L119).*

<a id="motorrole-line"></a>

### `MotorRole::line`

```cpp
std::uint32_t line = 0
```

The line of that `.name = ...` (exact: GCC 13 captures an initializer list at the declaration's closing line, so every role points at the `.name` line instead — measured).

*field, declared at [`include/shulib/mechanism/spec.hpp:122`](../../include/shulib/mechanism/spec.hpp#L122).*

<a id="struct-sensorrole"></a>

## `struct SensorRole`

```cpp
struct SensorRole
```

One SENSOR role. Only the fields of its kind are read.

*struct, declared at [`include/shulib/mechanism/spec.hpp:126`](../../include/shulib/mechanism/spec.hpp#L126).*

<a id="sensorrole-kind"></a>

### `SensorRole::kind`

```cpp
SensorKind kind = SensorKind::None
```

What it is; None = the mechanism has no such role.

*field, declared at [`include/shulib/mechanism/spec.hpp:128`](../../include/shulib/mechanism/spec.hpp#L128).*

<a id="sensorrole-reversed"></a>

### `SensorRole::reversed`

```cpp
bool reversed = false
```

Rotation: mounted so that + reads the other way.

*field, declared at [`include/shulib/mechanism/spec.hpp:130`](../../include/shulib/mechanism/spec.hpp#L130).*

<a id="sensorrole-pressedlevel"></a>

### `SensorRole::pressedLevel`

```cpp
bool pressedLevel = true
```

Switch: the level that means PRESSED. `true` (1) everywhere today — that a VEX limit switch reads 1 when pressed is HA-184's claim.

*field, declared at [`include/shulib/mechanism/spec.hpp:133`](../../include/shulib/mechanism/spec.hpp#L133).*

<a id="sensorrole-seeswithinin"></a>

### `SensorRole::seesWithinIn`

```cpp
double seesWithinIn = 0.0
```

Distance: an object closer than this many inches "sees a piece" (> 0).

*field, declared at [`include/shulib/mechanism/spec.hpp:135`](../../include/shulib/mechanism/spec.hpp#L135).*

<a id="sensorrole-seesproximity"></a>

### `SensorRole::seesProximity`

```cpp
double seesProximity = 0.0
```

Optical: a proximity at or above this "sees a piece", in (0, 1].

*field, declared at [`include/shulib/mechanism/spec.hpp:137`](../../include/shulib/mechanism/spec.hpp#L137).*

<a id="sensorrole-source"></a>

### `SensorRole::source`

```cpp
const char* source = nullptr
```

Where the role is stated in a GENERATED robot file (MotorRole::source); nullptr otherwise.

*field, declared at [`include/shulib/mechanism/spec.hpp:139`](../../include/shulib/mechanism/spec.hpp#L139).*

<a id="sensorrole-file"></a>

### `SensorRole::file`

```cpp
const char* file = nullptr
```

The file the role's word (rotationSensor(), distanceSensor(), ...) was written in.

*field, declared at [`include/shulib/mechanism/spec.hpp:141`](../../include/shulib/mechanism/spec.hpp#L141).*

<a id="sensorrole-line"></a>

### `SensorRole::line`

```cpp
std::uint32_t line = 0
```

The line it was written on.

*field, declared at [`include/shulib/mechanism/spec.hpp:143`](../../include/shulib/mechanism/spec.hpp#L143).*

<a id="enum-class-units"></a>

## `enum class Units`

```cpp
enum class Units : std::uint8_t
```

A position mechanism's UNITS. The byte 0 is UNSET.

*enum class, declared at [`include/shulib/mechanism/spec.hpp:147`](../../include/shulib/mechanism/spec.hpp#L147).*

<a id="units-degrees"></a>

### `Units::Degrees`

```cpp
Degrees = 1
```

degrees of the output shaft (an Arm)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:148`](../../include/shulib/mechanism/spec.hpp#L148).*

<a id="units-inches"></a>

### `Units::Inches`

```cpp
Inches = 2
```

inches of travel (a Lift)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:149`](../../include/shulib/mechanism/spec.hpp#L149).*

<a id="enum-class-geometry"></a>

## `enum class Geometry`

```cpp
enum class Geometry : std::uint8_t
```

How a position mechanism's unit relates to its output shaft's angle. The byte 0 is UNSET.

*enum class, declared at [`include/shulib/mechanism/spec.hpp:153`](../../include/shulib/mechanism/spec.hpp#L153).*

<a id="geometry-rotary"></a>

### `Geometry::Rotary`

```cpp
Rotary = 1
```

degrees: the output angle itself (an arm)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:154`](../../include/shulib/mechanism/spec.hpp#L154).*

<a id="geometry-spool"></a>

### `Geometry::Spool`

```cpp
Spool = 2
```

inches: a spool of `spoolDiameterIn` on the output shaft

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:155`](../../include/shulib/mechanism/spec.hpp#L155).*

<a id="geometry-arm"></a>

### `Geometry::Arm`

```cpp
Arm = 3
```

inches: height = pivotHeightIn + armStages * armLengthIn * sin(angle from level)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:156`](../../include/shulib/mechanism/spec.hpp#L156).*

<a id="geometry-table"></a>

### `Geometry::Table`

```cpp
Table = 4
```

inches: a measured table of (height in, output angle deg), strictly increasing

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:157`](../../include/shulib/mechanism/spec.hpp#L157).*

<a id="enum-class-gravity"></a>

## `enum class Gravity`

```cpp
enum class Gravity : std::uint8_t
```

What holds a position mechanism up. The byte 0 is UNSET.

*enum class, declared at [`include/shulib/mechanism/spec.hpp:161`](../../include/shulib/mechanism/spec.hpp#L161).*

<a id="gravity-none"></a>

### `Gravity::None`

```cpp
None = 1
```

nothing to hold: a turntable, a counterbalanced arm

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:162`](../../include/shulib/mechanism/spec.hpp#L162).*

<a id="gravity-cosine"></a>

### `Gravity::Cosine`

```cpp
Cosine = 2
```

an arm: kG * cos(angle from level)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:163`](../../include/shulib/mechanism/spec.hpp#L163).*

<a id="gravity-constant"></a>

### `Gravity::Constant`

```cpp
Constant = 3
```

a lift: kG everywhere

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:164`](../../include/shulib/mechanism/spec.hpp#L164).*

<a id="enum-class-homemethod"></a>

## `enum class HomeMethod`

```cpp
enum class HomeMethod : std::uint8_t
```

How a position mechanism finds its zero. The byte 0 is UNSET.

*enum class, declared at [`include/shulib/mechanism/spec.hpp:168`](../../include/shulib/mechanism/spec.hpp#L168).*

<a id="homemethod-none"></a>

### `HomeMethod::None`

```cpp
None = 1
```

never homed: position moves are refused (NOT_HOMED); a held button jogs

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:169`](../../include/shulib/mechanism/spec.hpp#L169).*

<a id="homemethod-restsatstop"></a>

### `HomeMethod::RestsAtStop`

```cpp
RestsAtStop = 2
```

it rests on its hard stop at program start: zero at bind, no motion

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:170`](../../include/shulib/mechanism/spec.hpp#L170).*

<a id="homemethod-switch"></a>

### `HomeMethod::Switch`

```cpp
Switch = 3
```

push at homeVolts until its home switch is pressed

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:171`](../../include/shulib/mechanism/spec.hpp#L171).*

<a id="homemethod-stall"></a>

### `HomeMethod::Stall`

```cpp
Stall = 4
```

push at homeVolts until its homing stall signature

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:172`](../../include/shulib/mechanism/spec.hpp#L172).*

<a id="homemethod-absolute"></a>

### `HomeMethod::Absolute`

```cpp
Absolute = 5
```

an absolute rotation sensor with a stated reading: known at bind (HA-186)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:173`](../../include/shulib/mechanism/spec.hpp#L173).*

<a id="struct-preset"></a>

## `struct Preset`

```cpp
struct Preset
```

One named preset of a position or speed mechanism.

*struct, declared at [`include/shulib/mechanism/spec.hpp:177`](../../include/shulib/mechanism/spec.hpp#L177).*

<a id="preset-name"></a>

### `Preset::name`

```cpp
char name[16]{}
```

Its name (15 characters at most, NUL-terminated): "score".

*field, declared at [`include/shulib/mechanism/spec.hpp:179`](../../include/shulib/mechanism/spec.hpp#L179).*

<a id="preset-value"></a>

### `Preset::value`

```cpp
double value = 0.0
```

Its value, in the mechanism's units (degrees, inches, or output rpm).

*field, declared at [`include/shulib/mechanism/spec.hpp:181`](../../include/shulib/mechanism/spec.hpp#L181).*

<a id="struct-stallspec"></a>

## `struct StallSpec`

```cpp
struct StallSpec
```

A STALL SIGNATURE as data — manipulation::StallConfig's three numbers. All 0 = none written. A WRITTEN one (any of the three) must be whole and able to trip (specRefusal: amps above 0 and below the motor's 2.5 A limit, an rpm above 0, a lasting above 0).

*struct, declared at [`include/shulib/mechanism/spec.hpp:187`](../../include/shulib/mechanism/spec.hpp#L187).*

<a id="stallspec-amps"></a>

### `StallSpec::amps`

```cpp
double amps = 0.0
```

At least this current (A) ...

*field, declared at [`include/shulib/mechanism/spec.hpp:189`](../../include/shulib/mechanism/spec.hpp#L189).*

<a id="stallspec-speedrads"></a>

### `StallSpec::speedRadS`

```cpp
double speedRadS = 0.0
```

... while the MOTOR shaft turns slower than this (rad/s) ...

*field, declared at [`include/shulib/mechanism/spec.hpp:191`](../../include/shulib/mechanism/spec.hpp#L191).*

<a id="stallspec-time"></a>

### `StallSpec::time`

```cpp
double time = 0.0
```

... continuously for this long (s).

*field, declared at [`include/shulib/mechanism/spec.hpp:193`](../../include/shulib/mechanism/spec.hpp#L193).*

<a id="stallspec-stated"></a>

### `StallSpec::stated`

```cpp
[[nodiscard]] constexpr bool stated() const noexcept
```

Whether a signature was stated: its amps (a whole one — specRefusal refuses a written one without amps).

*function, declared at [`include/shulib/mechanism/spec.hpp:196`](../../include/shulib/mechanism/spec.hpp#L196).*

<a id="stallspec-written"></a>

### `StallSpec::written`

```cpp
[[nodiscard]] constexpr bool written() const noexcept
```

Whether ANY of the three was written — what specRefusal checks a signature on, so one with a number left out is refused rather than read as "no signature".

*function, declared at [`include/shulib/mechanism/spec.hpp:199`](../../include/shulib/mechanism/spec.hpp#L199).*

<a id="struct-powerspec"></a>

## `struct PowerSpec`

```cpp
struct PowerSpec
```

A POWER mechanism's numbers.

*struct, declared at [`include/shulib/mechanism/spec.hpp:205`](../../include/shulib/mechanism/spec.hpp#L205).*

<a id="powerspec-maxvolts"></a>

### `PowerSpec::maxVolts`

```cpp
double maxVolts = 0.0
```

The most any command may ask, (0, 12] V.

*field, declared at [`include/shulib/mechanism/spec.hpp:207`](../../include/shulib/mechanism/spec.hpp#L207).*

<a id="powerspec-stall"></a>

### `PowerSpec::stall`

```cpp
StallSpec stall{}
```

Its jam signature — OPTIONAL on a power mechanism (every step is bounded by its time and the driver's hold-to-run by the finger; N10).

*field, declared at [`include/shulib/mechanism/spec.hpp:210`](../../include/shulib/mechanism/spec.hpp#L210).*

<a id="struct-positionspec"></a>

## `struct PositionSpec`

```cpp
struct PositionSpec
```

A POSITION mechanism's numbers (an Arm or a Lift).

*struct, declared at [`include/shulib/mechanism/spec.hpp:214`](../../include/shulib/mechanism/spec.hpp#L214).*

<a id="positionspec-units"></a>

### `PositionSpec::units`

```cpp
Units units = static_cast<Units>(0)
```

Degrees or inches. UNSET refused.

*field, declared at [`include/shulib/mechanism/spec.hpp:216`](../../include/shulib/mechanism/spec.hpp#L216).*

<a id="positionspec-geometry"></a>

### `PositionSpec::geometry`

```cpp
Geometry geometry = static_cast<Geometry>(0)
```

How the units relate to the output angle. UNSET refused.

*field, declared at [`include/shulib/mechanism/spec.hpp:218`](../../include/shulib/mechanism/spec.hpp#L218).*

<a id="positionspec-gearratio"></a>

### `PositionSpec::gearRatio`

```cpp
double gearRatio = 0.0
```

MOTOR turns per OUTPUT turn (> 0): gears(12, 60) is 5 (kinds.hpp — tooth counts, never a bare ratio, whose direction the drive geometry states the other way round).

*field, declared at [`include/shulib/mechanism/spec.hpp:221`](../../include/shulib/mechanism/spec.hpp#L221).*

<a id="positionspec-spooldiameterin"></a>

### `PositionSpec::spoolDiameterIn`

```cpp
double spoolDiameterIn = 0.0
```

Spool: its diameter, inches.

*field, declared at [`include/shulib/mechanism/spec.hpp:223`](../../include/shulib/mechanism/spec.hpp#L223).*

<a id="positionspec-armlengthin"></a>

### `PositionSpec::armLengthIn`

```cpp
double armLengthIn = 0.0
```

Arm (a linkage): one stage's length, pivot to pivot, inches.

*field, declared at [`include/shulib/mechanism/spec.hpp:225`](../../include/shulib/mechanism/spec.hpp#L225).*

<a id="positionspec-pivotheightin"></a>

### `PositionSpec::pivotHeightIn`

```cpp
double pivotHeightIn = 0.0
```

Arm: the first pivot's height, inches.

*field, declared at [`include/shulib/mechanism/spec.hpp:227`](../../include/shulib/mechanism/spec.hpp#L227).*

<a id="positionspec-armstages"></a>

### `PositionSpec::armStages`

```cpp
std::uint8_t armStages = 0
```

Arm: 1 = a single arm or a four-bar; 2 = a double reverse four-bar; 3 the most (0 = UNSET).

*field, declared at [`include/shulib/mechanism/spec.hpp:229`](../../include/shulib/mechanism/spec.hpp#L229).*

<a id="positionspec-table"></a>

### `PositionSpec::table`

```cpp
double table[kMaxTablePoints][2]{}
```

Table: {height in, output angle deg} rows, strictly increasing in both.

*field, declared at [`include/shulib/mechanism/spec.hpp:231`](../../include/shulib/mechanism/spec.hpp#L231).*

<a id="positionspec-tablepoints"></a>

### `PositionSpec::tablePoints`

```cpp
std::uint8_t tablePoints = 0
```

Table: how many rows of `table` are real.

*field, declared at [`include/shulib/mechanism/spec.hpp:233`](../../include/shulib/mechanism/spec.hpp#L233).*

<a id="positionspec-minposition"></a>

### `PositionSpec::minPosition`

```cpp
double minPosition = 0.0
```

The travel limits, in the mechanism's units (REQUIRED, min < max).

*field, declared at [`include/shulib/mechanism/spec.hpp:235`](../../include/shulib/mechanism/spec.hpp#L235).*

<a id="positionspec-maxposition"></a>

### `PositionSpec::maxPosition`

```cpp
double maxPosition = 0.0
```

The upper travel limit.

*field, declared at [`include/shulib/mechanism/spec.hpp:237`](../../include/shulib/mechanism/spec.hpp#L237).*

<a id="positionspec-maxvelocity"></a>

### `PositionSpec::maxVelocity`

```cpp
double maxVelocity = 0.0
```

The smooth move's top speed, units/s (> 0) — a STARTING value unless stated (header).

*field, declared at [`include/shulib/mechanism/spec.hpp:239`](../../include/shulib/mechanism/spec.hpp#L239).*

<a id="positionspec-maxacceleration"></a>

### `PositionSpec::maxAcceleration`

```cpp
double maxAcceleration = 0.0
```

The smooth move's acceleration, units/s^2 (> 0) — a starting value unless stated.

*field, declared at [`include/shulib/mechanism/spec.hpp:241`](../../include/shulib/mechanism/spec.hpp#L241).*

<a id="positionspec-tolerance"></a>

### `PositionSpec::tolerance`

```cpp
double tolerance = 0.0
```

"Reached": within this of the target, in the mechanism's units (> 0) ...

*field, declared at [`include/shulib/mechanism/spec.hpp:243`](../../include/shulib/mechanism/spec.hpp#L243).*

<a id="positionspec-settletime"></a>

### `PositionSpec::settleTime`

```cpp
double settleTime = 0.0
```

... continuously for this long, s (>= 0).

*field, declared at [`include/shulib/mechanism/spec.hpp:245`](../../include/shulib/mechanism/spec.hpp#L245).*

<a id="positionspec-kp"></a>

### `PositionSpec::kP`

```cpp
double kP = 0.0
```

Proportional gain, V per output degree — a STARTING value unless stated (header).

*field, declared at [`include/shulib/mechanism/spec.hpp:247`](../../include/shulib/mechanism/spec.hpp#L247).*

<a id="positionspec-ki"></a>

### `PositionSpec::kI`

```cpp
double kI = 0.0
```

Integral gain (the HOLD loop only), V per output degree-second.

*field, declared at [`include/shulib/mechanism/spec.hpp:249`](../../include/shulib/mechanism/spec.hpp#L249).*

<a id="positionspec-kd"></a>

### `PositionSpec::kD`

```cpp
double kD = 0.0
```

Derivative gain, V per output degree/s.

*field, declared at [`include/shulib/mechanism/spec.hpp:251`](../../include/shulib/mechanism/spec.hpp#L251).*

<a id="positionspec-ks"></a>

### `PositionSpec::kS`

```cpp
double kS = 0.0
```

Static friction feedforward, V (0 unless stated: the hold loop's integral covers it).

*field, declared at [`include/shulib/mechanism/spec.hpp:253`](../../include/shulib/mechanism/spec.hpp#L253).*

<a id="positionspec-kv"></a>

### `PositionSpec::kV`

```cpp
double kV = 0.0
```

Velocity feedforward, V per output degree/s (12 V over the free speed unless stated).

*field, declared at [`include/shulib/mechanism/spec.hpp:255`](../../include/shulib/mechanism/spec.hpp#L255).*

<a id="positionspec-ka"></a>

### `PositionSpec::kA`

```cpp
double kA = 0.0
```

Acceleration feedforward, V per output degree/s^2 (0 unless stated).

*field, declared at [`include/shulib/mechanism/spec.hpp:257`](../../include/shulib/mechanism/spec.hpp#L257).*

<a id="positionspec-kg"></a>

### `PositionSpec::kG`

```cpp
double kG = 0.0
```

The gravity hold, V: at level (Cosine) or everywhere (Constant). The TEAM's number, within +-12 V (negative for a mechanism its elastics pull up harder than gravity pulls down).

*field, declared at [`include/shulib/mechanism/spec.hpp:260`](../../include/shulib/mechanism/spec.hpp#L260).*

<a id="positionspec-gravity"></a>

### `PositionSpec::gravity`

```cpp
Gravity gravity = static_cast<Gravity>(0)
```

What gravity does to it. UNSET refused.

*field, declared at [`include/shulib/mechanism/spec.hpp:262`](../../include/shulib/mechanism/spec.hpp#L262).*

<a id="positionspec-horizontalat"></a>

### `PositionSpec::horizontalAt`

```cpp
double horizontalAt = 0.0
```

Cosine: the output angle (deg) at which the arm is level.

*field, declared at [`include/shulib/mechanism/spec.hpp:264`](../../include/shulib/mechanism/spec.hpp#L264).*

<a id="positionspec-home"></a>

### `PositionSpec::home`

```cpp
HomeMethod home = static_cast<HomeMethod>(0)
```

How it finds zero. UNSET refused.

*field, declared at [`include/shulib/mechanism/spec.hpp:266`](../../include/shulib/mechanism/spec.hpp#L266).*

<a id="positionspec-homeposition"></a>

### `PositionSpec::homePosition`

```cpp
double homePosition = 0.0
```

Where zero puts it, in its units (within the limits).

*field, declared at [`include/shulib/mechanism/spec.hpp:268`](../../include/shulib/mechanism/spec.hpp#L268).*

<a id="positionspec-homevolts"></a>

### `PositionSpec::homeVolts`

```cpp
double homeVolts = 0.0
```

Switch / Stall: the gentle push toward the stop, V, signed (it must point at the home end), within +-12 V.

*field, declared at [`include/shulib/mechanism/spec.hpp:271`](../../include/shulib/mechanism/spec.hpp#L271).*

<a id="positionspec-homestall"></a>

### `PositionSpec::homeStall`

```cpp
StallSpec homeStall{}
```

Stall homing: the signature that means "at the stop".

*field, declared at [`include/shulib/mechanism/spec.hpp:273`](../../include/shulib/mechanism/spec.hpp#L273).*

<a id="positionspec-stall"></a>

### `PositionSpec::stall`

```cpp
StallSpec stall{}
```

A blocked move's or hold's signature — REQUIRED (N10).

*field, declared at [`include/shulib/mechanism/spec.hpp:275`](../../include/shulib/mechanism/spec.hpp#L275).*

<a id="positionspec-unhomedvolts"></a>

### `PositionSpec::unhomedVolts`

```cpp
double unhomedVolts = 0.0
```

A hold-to-run before homing runs at most this, V, 0..12 (0 = nothing moves it before homing).

*field, declared at [`include/shulib/mechanism/spec.hpp:277`](../../include/shulib/mechanism/spec.hpp#L277).*

<a id="positionspec-absolutesensoratzerodeg"></a>

### `PositionSpec::absoluteSensorAtZeroDeg`

```cpp
double absoluteSensorAtZeroDeg = 0.0
```

Absolute: the angle sensor's reading (deg, [0, 360)) when the mechanism is at homePosition.

*field, declared at [`include/shulib/mechanism/spec.hpp:279`](../../include/shulib/mechanism/spec.hpp#L279).*

<a id="positionspec-presets"></a>

### `PositionSpec::presets`

```cpp
Preset presets[kMaxPresets]{}
```

Its named presets, in its units.

*field, declared at [`include/shulib/mechanism/spec.hpp:281`](../../include/shulib/mechanism/spec.hpp#L281).*

<a id="positionspec-presetcount"></a>

### `PositionSpec::presetCount`

```cpp
std::uint8_t presetCount = 0
```

How many of `presets` are real.

*field, declared at [`include/shulib/mechanism/spec.hpp:283`](../../include/shulib/mechanism/spec.hpp#L283).*

<a id="struct-speedspec"></a>

## `struct SpeedSpec`

```cpp
struct SpeedSpec
```

A SPEED mechanism's numbers (a Flywheel).

*struct, declared at [`include/shulib/mechanism/spec.hpp:287`](../../include/shulib/mechanism/spec.hpp#L287).*

<a id="speedspec-gearratio"></a>

### `SpeedSpec::gearRatio`

```cpp
double gearRatio = 0.0
```

MOTOR turns per OUTPUT turn (> 0): gears(72, 12) is 1/6 (geared up).

*field, declared at [`include/shulib/mechanism/spec.hpp:289`](../../include/shulib/mechanism/spec.hpp#L289).*

<a id="speedspec-maxrpm"></a>

### `SpeedSpec::maxRpm`

```cpp
double maxRpm = 0.0
```

The fastest target accepted, output rpm (> 0) — the cartridge's free speed through the gearing, derived by kinds.hpp, never typed.

*field, declared at [`include/shulib/mechanism/spec.hpp:292`](../../include/shulib/mechanism/spec.hpp#L292).*

<a id="speedspec-ks"></a>

### `SpeedSpec::kS`

```cpp
double kS = 0.0
```

Static feedforward, V.

*field, declared at [`include/shulib/mechanism/spec.hpp:294`](../../include/shulib/mechanism/spec.hpp#L294).*

<a id="speedspec-kv"></a>

### `SpeedSpec::kV`

```cpp
double kV = 0.0
```

Velocity feedforward, V per output rad/s (12 V over the free speed unless stated).

*field, declared at [`include/shulib/mechanism/spec.hpp:296`](../../include/shulib/mechanism/spec.hpp#L296).*

<a id="speedspec-ka"></a>

### `SpeedSpec::kA`

```cpp
double kA = 0.0
```

Acceleration feedforward, V per output rad/s^2.

*field, declared at [`include/shulib/mechanism/spec.hpp:298`](../../include/shulib/mechanism/spec.hpp#L298).*

<a id="speedspec-kp"></a>

### `SpeedSpec::kP`

```cpp
double kP = 0.0
```

Proportional gain on the speed error, V per rad/s — a STARTING value unless stated.

*field, declared at [`include/shulib/mechanism/spec.hpp:300`](../../include/shulib/mechanism/spec.hpp#L300).*

<a id="speedspec-ki"></a>

### `SpeedSpec::kI`

```cpp
double kI = 0.0
```

Integral gain on the speed error (acting near the target only — MECH-A3), V per rad.

*field, declared at [`include/shulib/mechanism/spec.hpp:302`](../../include/shulib/mechanism/spec.hpp#L302).*

<a id="speedspec-atspeedbandrpm"></a>

### `SpeedSpec::atSpeedBandRpm`

```cpp
double atSpeedBandRpm = 0.0
```

"At speed": within this many output rpm of the target ...

*field, declared at [`include/shulib/mechanism/spec.hpp:304`](../../include/shulib/mechanism/spec.hpp#L304).*

<a id="speedspec-atspeedtime"></a>

### `SpeedSpec::atSpeedTime`

```cpp
double atSpeedTime = 0.0
```

... continuously for this long, s (>= 0).

*field, declared at [`include/shulib/mechanism/spec.hpp:306`](../../include/shulib/mechanism/spec.hpp#L306).*

<a id="speedspec-presets"></a>

### `SpeedSpec::presets`

```cpp
Preset presets[kMaxPresets]{}
```

Its named speeds, output rpm ("shoot" = 3000).

*field, declared at [`include/shulib/mechanism/spec.hpp:308`](../../include/shulib/mechanism/spec.hpp#L308).*

<a id="speedspec-presetcount"></a>

### `SpeedSpec::presetCount`

```cpp
std::uint8_t presetCount = 0
```

How many of `presets` are real.

*field, declared at [`include/shulib/mechanism/spec.hpp:310`](../../include/shulib/mechanism/spec.hpp#L310).*

<a id="speedspec-stall"></a>

### `SpeedSpec::stall`

```cpp
StallSpec stall{}
```

A jammed wheel's signature — REQUIRED (N10).

*field, declared at [`include/shulib/mechanism/spec.hpp:312`](../../include/shulib/mechanism/spec.hpp#L312).*

<a id="enum-class-confirm"></a>

## `enum class Confirm`

```cpp
enum class Confirm : std::uint8_t
```

What proves a pneumatic position was reached (Q2): nothing (SUCCEEDED after the move time), or the piece sensor. A position that asks for the sensor on a mechanism WITHOUT one is counted done after the move time and reported UNCONFIRMED (the team lead's Q2 (c)).

*enum class, declared at [`include/shulib/mechanism/spec.hpp:318`](../../include/shulib/mechanism/spec.hpp#L318).*

<a id="confirm-none"></a>

### `Confirm::None`

```cpp
None = 0
```

nothing confirms it

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:319`](../../include/shulib/mechanism/spec.hpp#L319).*

<a id="confirm-sees"></a>

### `Confirm::Sees`

```cpp
Sees = 1
```

a GRAB: the piece sensor sees a piece

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:320`](../../include/shulib/mechanism/spec.hpp#L320).*

<a id="confirm-stopsseeing"></a>

### `Confirm::StopsSeeing`

```cpp
StopsSeeing = 2
```

a RELEASE: the piece sensor stops seeing one

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:321`](../../include/shulib/mechanism/spec.hpp#L321).*

<a id="struct-pneumaticpositionspec"></a>

## `struct PneumaticPositionSpec`

```cpp
struct PneumaticPositionSpec
```

One named position of a pneumatic mechanism.

*struct, declared at [`include/shulib/mechanism/spec.hpp:325`](../../include/shulib/mechanism/spec.hpp#L325).*

<a id="pneumaticpositionspec-name"></a>

### `PneumaticPositionSpec::name`

```cpp
char name[16]{}
```

Its name (15 characters at most): "closed".

*field, declared at [`include/shulib/mechanism/spec.hpp:327`](../../include/shulib/mechanism/spec.hpp#L327).*

<a id="pneumaticpositionspec-levels"></a>

### `PneumaticPositionSpec::levels`

```cpp
bool levels[kMaxLineRoles]{}
```

The level of each line in this position, line 1 first (Q13 (a): the team states them).

*field, declared at [`include/shulib/mechanism/spec.hpp:329`](../../include/shulib/mechanism/spec.hpp#L329).*

<a id="pneumaticpositionspec-confirm"></a>

### `PneumaticPositionSpec::confirm`

```cpp
Confirm confirm = Confirm::None
```

What confirms reaching it.

*field, declared at [`include/shulib/mechanism/spec.hpp:331`](../../include/shulib/mechanism/spec.hpp#L331).*

<a id="pneumaticpositionspec-levelcount"></a>

### `PneumaticPositionSpec::levelCount`

```cpp
std::uint8_t levelCount = 0
```

How many levels the team WROTE — the wiring refuses a count that is not its line count (U2).

*field, declared at [`include/shulib/mechanism/spec.hpp:333`](../../include/shulib/mechanism/spec.hpp#L333).*

<a id="struct-pneumaticspec"></a>

## `struct PneumaticSpec`

```cpp
struct PneumaticSpec
```

A PNEUMATIC mechanism's numbers.

*struct, declared at [`include/shulib/mechanism/spec.hpp:337`](../../include/shulib/mechanism/spec.hpp#L337).*

<a id="pneumaticspec-positions"></a>

### `PneumaticSpec::positions`

```cpp
PneumaticPositionSpec positions[kMaxPneumaticPositions]{}
```

Its positions, in the order written.

*field, declared at [`include/shulib/mechanism/spec.hpp:339`](../../include/shulib/mechanism/spec.hpp#L339).*

<a id="pneumaticspec-positioncount"></a>

### `PneumaticSpec::positionCount`

```cpp
std::uint8_t positionCount = 0
```

How many are real: 2 or 3.

*field, declared at [`include/shulib/mechanism/spec.hpp:341`](../../include/shulib/mechanism/spec.hpp#L341).*

<a id="pneumaticspec-start"></a>

### `PneumaticSpec::start`

```cpp
std::uint8_t start = 0
```

The position at program start (an index).

*field, declared at [`include/shulib/mechanism/spec.hpp:343`](../../include/shulib/mechanism/spec.hpp#L343).*

<a id="pneumaticspec-safe"></a>

### `PneumaticSpec::safe`

```cpp
std::uint8_t safe = 0
```

The position a forced safe state commands (an index).

*field, declared at [`include/shulib/mechanism/spec.hpp:345`](../../include/shulib/mechanism/spec.hpp#L345).*

<a id="pneumaticspec-actuationtime"></a>

### `PneumaticSpec::actuationTime`

```cpp
double actuationTime = 0.0
```

How long the cylinder takes, s (> 0; the team's — no default, P6).

*field, declared at [`include/shulib/mechanism/spec.hpp:347`](../../include/shulib/mechanism/spec.hpp#L347).*

<a id="pneumaticspec-confirmwindow"></a>

### `PneumaticSpec::confirmWindow`

```cpp
double confirmWindow = 0.0
```

With a piece sensor: how long after the move time a confirmation may take, s (>= 0).

*field, declared at [`include/shulib/mechanism/spec.hpp:349`](../../include/shulib/mechanism/spec.hpp#L349).*

<a id="pneumaticspec-levelssource"></a>

### `PneumaticSpec::levelsSource`

```cpp
const char* levelsSource = nullptr
```

WHERE the positions' levels are stated in a GENERATED robot file (its JSON path), for the direction check's "the levels are swapped: change them HERE"; nullptr in a hand-written file.

*field, declared at [`include/shulib/mechanism/spec.hpp:352`](../../include/shulib/mechanism/spec.hpp#L352).*

<a id="pneumaticspec-levelsfile"></a>

### `PneumaticSpec::levelsFile`

```cpp
const char* levelsFile = nullptr
```

The file the declaration was written in (captured at its `.name` line).

*field, declared at [`include/shulib/mechanism/spec.hpp:354`](../../include/shulib/mechanism/spec.hpp#L354).*

<a id="pneumaticspec-levelsline"></a>

### `PneumaticSpec::levelsLine`

```cpp
std::uint32_t levelsLine = 0
```

That line.

*field, declared at [`include/shulib/mechanism/spec.hpp:356`](../../include/shulib/mechanism/spec.hpp#L356).*

<a id="pneumaticspec-startnamed"></a>

### `PneumaticSpec::startNamed`

```cpp
bool startNamed = true
```

Whether the declaration's `start` named one of its positions.

*field, declared at [`include/shulib/mechanism/spec.hpp:358`](../../include/shulib/mechanism/spec.hpp#L358).*

<a id="pneumaticspec-safenamed"></a>

### `PneumaticSpec::safeNamed`

```cpp
bool safeNamed = true
```

Whether its `safe` named one of its positions.

*field, declared at [`include/shulib/mechanism/spec.hpp:360`](../../include/shulib/mechanism/spec.hpp#L360).*

<a id="struct-mechanismspec"></a>

## `struct MechanismSpec`

```cpp
struct MechanismSpec
```

ONE MECHANISM, as data (header).

*struct, declared at [`include/shulib/mechanism/spec.hpp:364`](../../include/shulib/mechanism/spec.hpp#L364).*

<a id="mechanismspec-name"></a>

### `MechanismSpec::name`

```cpp
char name[kNameBytes]{}
```

Its name — boot lines, logs, the route ids' stem, and the run-time lookup's key.

*field, declared at [`include/shulib/mechanism/spec.hpp:366`](../../include/shulib/mechanism/spec.hpp#L366).*

<a id="mechanismspec-kind"></a>

### `MechanismSpec::kind`

```cpp
Kind kind = static_cast<Kind>(0)
```

Which behaviour runs it. UNSET refused.

*field, declared at [`include/shulib/mechanism/spec.hpp:368`](../../include/shulib/mechanism/spec.hpp#L368).*

<a id="mechanismspec-motors"></a>

### `MechanismSpec::motors`

```cpp
MotorRole motors[kMaxMotorRoles]{}
```

Its motor roles, in order (a motor kind).

*field, declared at [`include/shulib/mechanism/spec.hpp:370`](../../include/shulib/mechanism/spec.hpp#L370).*

<a id="mechanismspec-motorcount"></a>

### `MechanismSpec::motorCount`

```cpp
std::uint8_t motorCount = 0
```

How many motor roles are real (1..4 for a motor kind; 0 for a pneumatic).

*field, declared at [`include/shulib/mechanism/spec.hpp:372`](../../include/shulib/mechanism/spec.hpp#L372).*

<a id="mechanismspec-linecount"></a>

### `MechanismSpec::lineCount`

```cpp
std::uint8_t lineCount = 0
```

How many 3-wire line roles (1..2 for a pneumatic; 0 otherwise) — the WIRING's count.

*field, declared at [`include/shulib/mechanism/spec.hpp:374`](../../include/shulib/mechanism/spec.hpp#L374).*

<a id="mechanismspec-anglesensor"></a>

### `MechanismSpec::angleSensor`

```cpp
SensorRole angleSensor{}
```

Position / Speed: a Rotation role on its output shaft.

*field, declared at [`include/shulib/mechanism/spec.hpp:376`](../../include/shulib/mechanism/spec.hpp#L376).*

<a id="mechanismspec-homeswitch"></a>

### `MechanismSpec::homeSwitch`

```cpp
SensorRole homeSwitch{}
```

Position: a Switch role (implied by switch homing).

*field, declared at [`include/shulib/mechanism/spec.hpp:378`](../../include/shulib/mechanism/spec.hpp#L378).*

<a id="mechanismspec-piecesensor"></a>

### `MechanismSpec::pieceSensor`

```cpp
SensorRole pieceSensor{}
```

Any kind: a Distance, Optical or Switch piece sensor.

*field, declared at [`include/shulib/mechanism/spec.hpp:380`](../../include/shulib/mechanism/spec.hpp#L380).*

<a id="mechanismspec-cartridge"></a>

### `MechanismSpec::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

A motor kind's cartridge — its own, never the drive's. UNSET refused.

*field, declared at [`include/shulib/mechanism/spec.hpp:382`](../../include/shulib/mechanism/spec.hpp#L382).*

<a id="mechanismspec-safemode"></a>

### `MechanismSpec::safeMode`

```cpp
config::SafeBrakeMode safeMode = static_cast<config::SafeBrakeMode>(0)
```

A motor kind's declared safe brake mode — what every stop lands in. UNSET refused.

*field, declared at [`include/shulib/mechanism/spec.hpp:384`](../../include/shulib/mechanism/spec.hpp#L384).*

<a id="mechanismspec-power"></a>

### `MechanismSpec::power`

```cpp
PowerSpec power{}
```

Power numbers (read when kind is Power).

*field, declared at [`include/shulib/mechanism/spec.hpp:386`](../../include/shulib/mechanism/spec.hpp#L386).*

<a id="mechanismspec-position"></a>

### `MechanismSpec::position`

```cpp
PositionSpec position{}
```

Position numbers (read when kind is Position).

*field, declared at [`include/shulib/mechanism/spec.hpp:388`](../../include/shulib/mechanism/spec.hpp#L388).*

<a id="mechanismspec-speed"></a>

### `MechanismSpec::speed`

```cpp
SpeedSpec speed{}
```

Speed numbers (read when kind is Speed).

*field, declared at [`include/shulib/mechanism/spec.hpp:390`](../../include/shulib/mechanism/spec.hpp#L390).*

<a id="mechanismspec-pneumatic"></a>

### `MechanismSpec::pneumatic`

```cpp
PneumaticSpec pneumatic{}
```

Pneumatic numbers (read when kind is Pneumatic).

*field, declared at [`include/shulib/mechanism/spec.hpp:392`](../../include/shulib/mechanism/spec.hpp#L392).*

<a id="mechanismspec-declaredfile"></a>

### `MechanismSpec::declaredFile`

```cpp
const char* declaredFile = nullptr
```

Where the declaration was written (its `.name = ...` line), for boot lines and the direction check — the path the compiler opened (pin a suffix, never the whole path).

*field, declared at [`include/shulib/mechanism/spec.hpp:395`](../../include/shulib/mechanism/spec.hpp#L395).*

<a id="mechanismspec-declaredline"></a>

### `MechanismSpec::declaredLine`

```cpp
std::uint32_t declaredLine = 0
```

That line.

*field, declared at [`include/shulib/mechanism/spec.hpp:397`](../../include/shulib/mechanism/spec.hpp#L397).*

<a id="mechanismspec-presetnametoolong"></a>

### `MechanismSpec::presetNameTooLong`

```cpp
bool presetNameTooLong = false
```

Set by kinds.hpp when a preset or position name runs past 15 characters (its copy was cut).

*field, declared at [`include/shulib/mechanism/spec.hpp:399`](../../include/shulib/mechanism/spec.hpp#L399).*

<a id="nameof"></a>

## `nameOf`

```cpp
[[nodiscard]] constexpr std::string_view nameOf(const MechanismSpec& s) noexcept
```

A spec's name as a view (up to its NUL, at most kNameBytes).

*free function, declared at [`include/shulib/mechanism/spec.hpp:403`](../../include/shulib/mechanism/spec.hpp#L403).*

<a id="angledegtoposition"></a>

## `angleDegToPosition`

```cpp
[[nodiscard]] inline double angleDegToPosition(const PositionSpec& p, double outDeg) noexcept
```

Output degrees -> the mechanism's units, for a Position spec. Rotary: the angle; Spool: arc length; Arm: height from the sine of the angle above level; Table: linear between rows, clamped at both ends. Pure; positionToAngleDeg is the inverse.

*free function, declared at [`include/shulib/mechanism/spec.hpp:414`](../../include/shulib/mechanism/spec.hpp#L414).*

<a id="positiontoangledeg"></a>

## `positionToAngleDeg`

```cpp
[[nodiscard]] inline double positionToAngleDeg(const PositionSpec& p, double pos) noexcept
```

The mechanism's units -> output degrees (the inverse of angleDegToPosition). Arm: asin, valid within +-90 deg of level — the asin argument is clamped, and specRefusal() refuses limits outside the formula's range, where the height is not monotonic. Table: linear, clamped at both ends.

*free function, declared at [`include/shulib/mechanism/spec.hpp:447`](../../include/shulib/mechanism/spec.hpp#L447).*

<a id="enum-class-refusal"></a>

## `enum class Refusal`

```cpp
enum class Refusal : std::uint8_t
```

EVERY reason the build refuses ONE mechanism's declaration — one property each, in the order specRefusal() checks them (header). APPENDED ONLY: G3a's validator mirrors this list by sentence.

*enum class, declared at [`include/shulib/mechanism/spec.hpp:481`](../../include/shulib/mechanism/spec.hpp#L481).*

<a id="refusal-none"></a>

### `Refusal::None`

```cpp
None = 0
```

nothing is wrong

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:482`](../../include/shulib/mechanism/spec.hpp#L482).*

<a id="refusal-amechanismhasnoname"></a>

### `Refusal::AMechanismHasNoName`

```cpp
AMechanismHasNoName
```

its name is empty

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:483`](../../include/shulib/mechanism/spec.hpp#L483).*

<a id="refusal-amechanismnamedoesnotfit23characters"></a>

### `Refusal::AMechanismNameDoesNotFit23Characters`

```cpp
AMechanismNameDoesNotFit23Characters
```

its name has no NUL inside 24 bytes

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:484`](../../include/shulib/mechanism/spec.hpp#L484).*

<a id="refusal-amechanismskindisunset"></a>

### `Refusal::AMechanismSKindIsUNSET`

```cpp
AMechanismSKindIsUNSET
```

its kind byte is none of the four

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:485`](../../include/shulib/mechanism/spec.hpp#L485).*

<a id="refusal-amotormechanismneeds14motorroles"></a>

### `Refusal::AMotorMechanismNeeds14MotorRoles`

```cpp
AMotorMechanismNeeds14MotorRoles
```

a motor kind with 0 or more than 4 motor roles

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:486`](../../include/shulib/mechanism/spec.hpp#L486).*

<a id="refusal-amotormechanismhas3wirelinerolesmaketwomechanisms"></a>

### `Refusal::AMotorMechanismHas3WireLineRolesMakeTwoMechanisms`

```cpp
AMotorMechanismHas3WireLineRolesMakeTwoMechanisms
```

a motor kind with 3-wire line roles

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:487`](../../include/shulib/mechanism/spec.hpp#L487).*

<a id="refusal-amotormechanismscartridgeisunset"></a>

### `Refusal::AMotorMechanismSCartridgeIsUNSET`

```cpp
AMotorMechanismSCartridgeIsUNSET
```

a motor kind with no cartridge

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:488`](../../include/shulib/mechanism/spec.hpp#L488).*

<a id="refusal-amotormechanismssafebrakemodeisunset"></a>

### `Refusal::AMotorMechanismSSafeBrakeModeIsUNSET`

```cpp
AMotorMechanismSSafeBrakeModeIsUNSET
```

a motor kind with no safe brake mode

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:489`](../../include/shulib/mechanism/spec.hpp#L489).*

<a id="refusal-apneumaticmechanismhasmotorroles"></a>

### `Refusal::APneumaticMechanismHasMotorRoles`

```cpp
APneumaticMechanismHasMotorRoles
```

a pneumatic with motor roles

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:490`](../../include/shulib/mechanism/spec.hpp#L490).*

<a id="refusal-apneumaticmechanismneeds12lineroles"></a>

### `Refusal::APneumaticMechanismNeeds12LineRoles`

```cpp
APneumaticMechanismNeeds12LineRoles
```

a pneumatic with 0 or more than 2 lines

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:491`](../../include/shulib/mechanism/spec.hpp#L491).*

<a id="refusal-theanglesensorroleisnotarotationsensor"></a>

### `Refusal::TheAngleSensorRoleIsNotARotationSensor`

```cpp
TheAngleSensorRoleIsNotARotationSensor
```

the angle-sensor slot holds another kind

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:492`](../../include/shulib/mechanism/spec.hpp#L492).*

<a id="refusal-ananglesensorroleonamechanismthatisnotpositionorspeed"></a>

### `Refusal::AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed`

```cpp
AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed
```

an angle sensor on a power or pneumatic

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:493`](../../include/shulib/mechanism/spec.hpp#L493).*

<a id="refusal-thehomeswitchroleisnotalimitswitch"></a>

### `Refusal::TheHomeSwitchRoleIsNotALimitSwitch`

```cpp
TheHomeSwitchRoleIsNotALimitSwitch
```

the home-switch slot holds another kind

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:494`](../../include/shulib/mechanism/spec.hpp#L494).*

<a id="refusal-ahomeswitchroleonamechanismthatisnotposition"></a>

### `Refusal::AHomeSwitchRoleOnAMechanismThatIsNotPosition`

```cpp
AHomeSwitchRoleOnAMechanismThatIsNotPosition
```

a home switch on a mechanism that is not position

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:495`](../../include/shulib/mechanism/spec.hpp#L495).*

<a id="refusal-apiecesensorcannotbearotationsensor"></a>

### `Refusal::APieceSensorCannotBeARotationSensor`

```cpp
APieceSensorCannotBeARotationSensor
```

a rotation sensor as a piece sensor

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:496`](../../include/shulib/mechanism/spec.hpp#L496).*

<a id="refusal-adistancepiecesensorneedsitsseeswithindistance0"></a>

### `Refusal::ADistancePieceSensorNeedsItsSeesWithinDistance0`

```cpp
ADistancePieceSensorNeedsItsSeesWithinDistance0
```

a distance piece sensor with no threshold

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:497`](../../include/shulib/mechanism/spec.hpp#L497).*

<a id="refusal-anopticalpiecesensorneedsitsproximitythresholdin01"></a>

### `Refusal::AnOpticalPieceSensorNeedsItsProximityThresholdIn01`

```cpp
AnOpticalPieceSensorNeedsItsProximityThresholdIn01
```

an optical piece sensor's threshold outside (0, 1]

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:498`](../../include/shulib/mechanism/spec.hpp#L498).*

<a id="refusal-powermaxvoltsisnotin012"></a>

### `Refusal::PowerMaxVoltsIsNotIn012`

```cpp
PowerMaxVoltsIsNotIn012
```

a power kind's maxVolts outside (0, 12]

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:499`](../../include/shulib/mechanism/spec.hpp#L499).*

<a id="refusal-positionunitsunset"></a>

### `Refusal::PositionUnitsUNSET`

```cpp
PositionUnitsUNSET
```

a position kind's units byte is unset

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:500`](../../include/shulib/mechanism/spec.hpp#L500).*

<a id="refusal-positiongearratiomustbe0"></a>

### `Refusal::PositionGearRatioMustBe0`

```cpp
PositionGearRatioMustBe0
```

a position kind with no gearing

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:501`](../../include/shulib/mechanism/spec.hpp#L501).*

<a id="refusal-positionarotarymechanismisindegrees"></a>

### `Refusal::PositionARotaryMechanismIsInDegrees`

```cpp
PositionARotaryMechanismIsInDegrees
```

a rotary geometry in inches

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:502`](../../include/shulib/mechanism/spec.hpp#L502).*

<a id="refusal-positionaliftisininches"></a>

### `Refusal::PositionALiftIsInInches`

```cpp
PositionALiftIsInInches
```

a lift geometry in degrees

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:503`](../../include/shulib/mechanism/spec.hpp#L503).*

<a id="refusal-positionspooldiametermustbe0"></a>

### `Refusal::PositionSpoolDiameterMustBe0`

```cpp
PositionSpoolDiameterMustBe0
```

a spool with no diameter

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:504`](../../include/shulib/mechanism/spec.hpp#L504).*

<a id="refusal-positionarmlengthmustbe0"></a>

### `Refusal::PositionArmLengthMustBe0`

```cpp
PositionArmLengthMustBe0
```

a linkage with no arm length

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:505`](../../include/shulib/mechanism/spec.hpp#L505).*

<a id="refusal-positionanarmneeds13stages"></a>

### `Refusal::PositionAnArmNeeds13Stages`

```cpp
PositionAnArmNeeds13Stages
```

a linkage with 0 or more than 3 stages

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:506`](../../include/shulib/mechanism/spec.hpp#L506).*

<a id="refusal-positionanarmstravellimitsreachpastitslinkagetheheightformul"></a>

### `Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul`

```cpp
PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul
```

a linkage whose limits reach past its formula

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:507`](../../include/shulib/mechanism/spec.hpp#L507).*

<a id="refusal-positionalookuptableneeds28points"></a>

### `Refusal::PositionALookupTableNeeds28Points`

```cpp
PositionALookupTableNeeds28Points
```

a measured table with fewer than 2 or more than 8 points

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:508`](../../include/shulib/mechanism/spec.hpp#L508).*

<a id="refusal-positionthelookuptableisnotstrictlyincreasing"></a>

### `Refusal::PositionTheLookupTableIsNotStrictlyIncreasing`

```cpp
PositionTheLookupTableIsNotStrictlyIncreasing
```

a measured table not increasing in both columns

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:509`](../../include/shulib/mechanism/spec.hpp#L509).*

<a id="refusal-positiontravellimitsmissingorminmax"></a>

### `Refusal::PositionTravelLimitsMissingOrMinMax`

```cpp
PositionTravelLimitsMissingOrMinMax
```

no travel limits, or min >= max

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:510`](../../include/shulib/mechanism/spec.hpp#L510).*

<a id="refusal-positiontheprofileneedsmaxvelocityandmaxacceleration0"></a>

### `Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0`

```cpp
PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0
```

a profile speed or acceleration that is not > 0

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:511`](../../include/shulib/mechanism/spec.hpp#L511).*

<a id="refusal-positiontolerancemustbe0"></a>

### `Refusal::PositionToleranceMustBe0`

```cpp
PositionToleranceMustBe0
```

a 'reached' tolerance that is not > 0

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:512`](../../include/shulib/mechanism/spec.hpp#L512).*

<a id="refusal-positiongravityunset"></a>

### `Refusal::PositionGravityUNSET`

```cpp
PositionGravityUNSET
```

a gravity kind byte that is unset

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:513`](../../include/shulib/mechanism/spec.hpp#L513).*

<a id="refusal-positionhomingmethodunset"></a>

### `Refusal::PositionHomingMethodUNSET`

```cpp
PositionHomingMethodUNSET
```

a homing method byte that is unset

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:514`](../../include/shulib/mechanism/spec.hpp#L514).*

<a id="refusal-positionhomingbyswitchneedsahomeswitchrole"></a>

### `Refusal::PositionHomingBySwitchNeedsAHomeSwitchRole`

```cpp
PositionHomingBySwitchNeedsAHomeSwitchRole
```

switch homing with no home switch

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:515`](../../include/shulib/mechanism/spec.hpp#L515).*

<a id="refusal-positionabsolutehomingneedsananglesensorrotationrole"></a>

### `Refusal::PositionAbsoluteHomingNeedsAnAngleSensorRotationRole`

```cpp
PositionAbsoluteHomingNeedsAnAngleSensorRotationRole
```

absolute homing with no angle sensor

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:516`](../../include/shulib/mechanism/spec.hpp#L516).*

<a id="refusal-positionhomingneedshomevolts"></a>

### `Refusal::PositionHomingNeedsHomeVolts`

```cpp
PositionHomingNeedsHomeVolts
```

a homing move with no push

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:517`](../../include/shulib/mechanism/spec.hpp#L517).*

<a id="refusal-positionstallhomingneedsitsstallsignature"></a>

### `Refusal::PositionStallHomingNeedsItsStallSignature`

```cpp
PositionStallHomingNeedsItsStallSignature
```

stall homing with no signature

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:518`](../../include/shulib/mechanism/spec.hpp#L518).*

<a id="refusal-positionthehomepositionisoutsidethetravellimits"></a>

### `Refusal::PositionTheHomePositionIsOutsideTheTravelLimits`

```cpp
PositionTheHomePositionIsOutsideTheTravelLimits
```

a home position past the limits

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:519`](../../include/shulib/mechanism/spec.hpp#L519).*

<a id="refusal-positionmorepresetsthanfit"></a>

### `Refusal::PositionMorePresetsThanFit`

```cpp
PositionMorePresetsThanFit
```

more presets than kMaxPresets

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:520`](../../include/shulib/mechanism/spec.hpp#L520).*

<a id="refusal-positionapresethasnoname"></a>

### `Refusal::PositionAPresetHasNoName`

```cpp
PositionAPresetHasNoName
```

a position preset with no name

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:521`](../../include/shulib/mechanism/spec.hpp#L521).*

<a id="refusal-positionapresetisoutsidethetravellimits"></a>

### `Refusal::PositionAPresetIsOutsideTheTravelLimits`

```cpp
PositionAPresetIsOutsideTheTravelLimits
```

a position preset past the limits

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:522`](../../include/shulib/mechanism/spec.hpp#L522).*

<a id="refusal-speedgearratioandmaxrpmmustbe0"></a>

### `Refusal::SpeedGearRatioAndMaxRpmMustBe0`

```cpp
SpeedGearRatioAndMaxRpmMustBe0
```

a speed kind with no gearing or free speed

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:523`](../../include/shulib/mechanism/spec.hpp#L523).*

<a id="refusal-speedtheatspeedbandmustbe0"></a>

### `Refusal::SpeedTheAtSpeedBandMustBe0`

```cpp
SpeedTheAtSpeedBandMustBe0
```

a speed kind with no at-speed band

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:524`](../../include/shulib/mechanism/spec.hpp#L524).*

<a id="refusal-speedmorepresetsthanfit"></a>

### `Refusal::SpeedMorePresetsThanFit`

```cpp
SpeedMorePresetsThanFit
```

more speed presets than kMaxPresets

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:525`](../../include/shulib/mechanism/spec.hpp#L525).*

<a id="refusal-speedapresethasnoname"></a>

### `Refusal::SpeedAPresetHasNoName`

```cpp
SpeedAPresetHasNoName
```

a speed preset with no name

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:526`](../../include/shulib/mechanism/spec.hpp#L526).*

<a id="refusal-speedapresetispastmaxrpm"></a>

### `Refusal::SpeedAPresetIsPastMaxRpm`

```cpp
SpeedAPresetIsPastMaxRpm
```

a speed preset past the free speed

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:527`](../../include/shulib/mechanism/spec.hpp#L527).*

<a id="refusal-pneumatic2or3positions"></a>

### `Refusal::Pneumatic2Or3Positions`

```cpp
Pneumatic2Or3Positions
```

a pneumatic with fewer than 2 or more than 3 positions

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:528`](../../include/shulib/mechanism/spec.hpp#L528).*

<a id="refusal-pneumaticstartsafeisnotoneofitspositions"></a>

### `Refusal::PneumaticStartSafeIsNotOneOfItsPositions`

```cpp
PneumaticStartSafeIsNotOneOfItsPositions
```

a start or safe index past the positions

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:529`](../../include/shulib/mechanism/spec.hpp#L529).*

<a id="refusal-pneumaticactuationtimemustbe0"></a>

### `Refusal::PneumaticActuationTimeMustBe0`

```cpp
PneumaticActuationTimeMustBe0
```

no move time

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:530`](../../include/shulib/mechanism/spec.hpp#L530).*

<a id="refusal-pneumaticapositionhasnoname"></a>

### `Refusal::PneumaticAPositionHasNoName`

```cpp
PneumaticAPositionHasNoName
```

a pneumatic position with no name

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:531`](../../include/shulib/mechanism/spec.hpp#L531).*

<a id="refusal-pneumatictwopositionshavethesamelinelevels"></a>

### `Refusal::PneumaticTwoPositionsHaveTheSameLineLevels`

```cpp
PneumaticTwoPositionsHaveTheSameLineLevels
```

two positions alike on every wired line

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:532`](../../include/shulib/mechanism/spec.hpp#L532).*

<a id="refusal-pneumaticitsstartnamesnoneofitspositions"></a>

### `Refusal::PneumaticItsStartNamesNoneOfItsPositions`

```cpp
PneumaticItsStartNamesNoneOfItsPositions
```

a `start` naming no position

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:533`](../../include/shulib/mechanism/spec.hpp#L533).*

<a id="refusal-pneumaticitssafenamesnoneofitspositions"></a>

### `Refusal::PneumaticItsSafeNamesNoneOfItsPositions`

```cpp
PneumaticItsSafeNamesNoneOfItsPositions
```

a `safe` naming no position

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:534`](../../include/shulib/mechanism/spec.hpp#L534).*

<a id="refusal-apresetorpositionnamedoesnotfit15characters"></a>

### `Refusal::APresetOrPositionNameDoesNotFit15Characters`

```cpp
APresetOrPositionNameDoesNotFit15Characters
```

a preset or position name past 15 characters

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:535`](../../include/shulib/mechanism/spec.hpp#L535).*

<a id="refusal-twopresetsorpositionsshareaname"></a>

### `Refusal::TwoPresetsOrPositionsShareAName`

```cpp
TwoPresetsOrPositionsShareAName
```

two presets (or positions) with one name

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:536`](../../include/shulib/mechanism/spec.hpp#L536).*

<a id="refusal-absolutehomingthesensorsreadingatzeromustbein0360"></a>

### `Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360`

```cpp
AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360
```

an absolute sensor reading outside [0, 360)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:537`](../../include/shulib/mechanism/spec.hpp#L537).*

<a id="refusal-positionthestallsignatureisunset"></a>

### `Refusal::PositionTheStallSignatureIsUNSET`

```cpp
PositionTheStallSignatureIsUNSET
```

an arm or a lift with no stall signature (N10)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:538`](../../include/shulib/mechanism/spec.hpp#L538).*

<a id="refusal-positionhomingpushesawayfromitshomeend"></a>

### `Refusal::PositionHomingPushesAwayFromItsHomeEnd`

```cpp
PositionHomingPushesAwayFromItsHomeEnd
```

a homing push pointing away from its home end (U4)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:539`](../../include/shulib/mechanism/spec.hpp#L539).*

<a id="refusal-speedthestallsignatureisunset"></a>

### `Refusal::SpeedTheStallSignatureIsUNSET`

```cpp
SpeedTheStallSignatureIsUNSET
```

a flywheel with no stall signature (N10)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:540`](../../include/shulib/mechanism/spec.hpp#L540).*

<a id="refusal-astallsignaturesampsmustbebelowthemotors25acurrentlimit"></a>

### `Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit`

```cpp
AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit
```

a stated signature at or above 2.5 A (N10: it could never trip)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:541`](../../include/shulib/mechanism/spec.hpp#L541).*

<a id="refusal-astallsignatureneedsbelowmotorrpmabove0"></a>

### `Refusal::AStallSignatureNeedsBelowMotorRpmAbove0`

```cpp
AStallSignatureNeedsBelowMotorRpmAbove0
```

a stated signature at 0 rpm (N10: it could never trip)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:542`](../../include/shulib/mechanism/spec.hpp#L542).*

<a id="refusal-astallsignatureneedslastingabove0"></a>

### `Refusal::AStallSignatureNeedsLastingAbove0`

```cpp
AStallSignatureNeedsLastingAbove0
```

a stated signature with no `.lasting` (MECH-A1: it would trip on every start)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:543`](../../include/shulib/mechanism/spec.hpp#L543).*

<a id="refusal-astallsignatureneedsampsabove0"></a>

### `Refusal::AStallSignatureNeedsAmpsAbove0`

```cpp
AStallSignatureNeedsAmpsAbove0
```

a WRITTEN signature (rpm or lasting) with no amps (MECH-A1)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:544`](../../include/shulib/mechanism/spec.hpp#L544).*

<a id="refusal-positiongeometryunset"></a>

### `Refusal::PositionGeometryUNSET`

```cpp
PositionGeometryUNSET
```

a lift with no .shape (MECH-A1)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:545`](../../include/shulib/mechanism/spec.hpp#L545).*

<a id="refusal-positionthesettletimeisnegative"></a>

### `Refusal::PositionTheSettleTimeIsNegative`

```cpp
PositionTheSettleTimeIsNegative
```

a negative settle time (MECH-A1 review: it was named as the tolerance)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:546`](../../include/shulib/mechanism/spec.hpp#L546).*

<a id="refusal-speedtheatspeedlastingisnegative"></a>

### `Refusal::SpeedTheAtSpeedLastingIsNegative`

```cpp
SpeedTheAtSpeedLastingIsNegative
```

a negative at-speed lasting (MECH-A1 review: it was named as the band)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:547`](../../include/shulib/mechanism/spec.hpp#L547).*

<a id="refusal-pneumatictheconfirmwindowisnegative"></a>

### `Refusal::PneumaticTheConfirmWindowIsNegative`

```cpp
PneumaticTheConfirmWindowIsNegative
```

a negative confirmWithin (MECH-A1 review: it was named as the move time)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:548`](../../include/shulib/mechanism/spec.hpp#L548).*

<a id="refusal-positionthehomingpushispast12volts"></a>

### `Refusal::PositionTheHomingPushIsPast12Volts`

```cpp
PositionTheHomingPushIsPast12Volts
```

a homing push past 12 V either way (MECH-A1 review)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:549`](../../include/shulib/mechanism/spec.hpp#L549).*

<a id="refusal-positionthegravityholdispast12volts"></a>

### `Refusal::PositionTheGravityHoldIsPast12Volts`

```cpp
PositionTheGravityHoldIsPast12Volts
```

a gravity hold past 12 V either way (MECH-A1 review)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:550`](../../include/shulib/mechanism/spec.hpp#L550).*

<a id="refusal-positionunhomedvoltsisnotin0to12"></a>

### `Refusal::PositionUnhomedVoltsIsNotIn0To12`

```cpp
PositionUnhomedVoltsIsNotIn0To12
```

an unhomedVolts outside [0, 12] (MECH-A1 review)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:551`](../../include/shulib/mechanism/spec.hpp#L551).*

<a id="refusal-positionabsolutehomingtravelspansawholesensorturn"></a>

### `Refusal::PositionAbsoluteHomingTravelSpansAWholeSensorTurn`

```cpp
PositionAbsoluteHomingTravelSpansAWholeSensorTurn
```

absolute homing over a travel of a whole output turn or more (MECH-A3: a reading within one turn cannot tell the turns apart)

*enumerator, declared at [`include/shulib/mechanism/spec.hpp:552`](../../include/shulib/mechanism/spec.hpp#L552).*

<a id="refusaltext"></a>

## `refusalText`

```cpp
[[nodiscard]] constexpr const char* refusalText(Refusal r) noexcept
```

The SENTENCE for a refusal — what a run-time caller logs, a host test pins, and the robot panel's validator mirrors (G3a). nullptr for None.

*free function, declared at [`include/shulib/mechanism/spec.hpp:557`](../../include/shulib/mechanism/spec.hpp#L557).*

<a id="a_mechanism_has_no_name"></a>

## `a_mechanism_has_no_name`

```cpp
inline void a_mechanism_has_no_name()
```

Refusal::AMechanismHasNoName's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:710`](../../include/shulib/mechanism/spec.hpp#L710).*

<a id="a_mechanism_name_does_not_fit_twenty_three_characters"></a>

## `a_mechanism_name_does_not_fit_twenty_three_characters`

```cpp
inline void a_mechanism_name_does_not_fit_twenty_three_characters()
```

Refusal::AMechanismNameDoesNotFit23Characters's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:712`](../../include/shulib/mechanism/spec.hpp#L712).*

<a id="a_mechanisms_kind_is_unset"></a>

## `a_mechanisms_kind_is_unset`

```cpp
inline void a_mechanisms_kind_is_unset()
```

Refusal::AMechanismSKindIsUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:714`](../../include/shulib/mechanism/spec.hpp#L714).*

<a id="a_motor_mechanism_needs_one_to_four_motor_roles"></a>

## `a_motor_mechanism_needs_one_to_four_motor_roles`

```cpp
inline void a_motor_mechanism_needs_one_to_four_motor_roles()
```

Refusal::AMotorMechanismNeeds14MotorRoles's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:716`](../../include/shulib/mechanism/spec.hpp#L716).*

<a id="a_motor_mechanism_has_three_wire_line_roles_make_two_mechanisms"></a>

## `a_motor_mechanism_has_three_wire_line_roles_make_two_mechanisms`

```cpp
inline void a_motor_mechanism_has_three_wire_line_roles_make_two_mechanisms()
```

Refusal::AMotorMechanismHas3WireLineRolesMakeTwoMechanisms's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:718`](../../include/shulib/mechanism/spec.hpp#L718).*

<a id="a_motor_mechanisms_cartridge_is_unset"></a>

## `a_motor_mechanisms_cartridge_is_unset`

```cpp
inline void a_motor_mechanisms_cartridge_is_unset()
```

Refusal::AMotorMechanismSCartridgeIsUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:720`](../../include/shulib/mechanism/spec.hpp#L720).*

<a id="a_motor_mechanisms_safe_brake_mode_is_unset"></a>

## `a_motor_mechanisms_safe_brake_mode_is_unset`

```cpp
inline void a_motor_mechanisms_safe_brake_mode_is_unset()
```

Refusal::AMotorMechanismSSafeBrakeModeIsUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:722`](../../include/shulib/mechanism/spec.hpp#L722).*

<a id="a_pneumatic_mechanism_has_motor_roles"></a>

## `a_pneumatic_mechanism_has_motor_roles`

```cpp
inline void a_pneumatic_mechanism_has_motor_roles()
```

Refusal::APneumaticMechanismHasMotorRoles's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:724`](../../include/shulib/mechanism/spec.hpp#L724).*

<a id="a_pneumatic_mechanism_needs_one_or_two_line_roles"></a>

## `a_pneumatic_mechanism_needs_one_or_two_line_roles`

```cpp
inline void a_pneumatic_mechanism_needs_one_or_two_line_roles()
```

Refusal::APneumaticMechanismNeeds12LineRoles's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:726`](../../include/shulib/mechanism/spec.hpp#L726).*

<a id="the_angle_sensor_role_is_not_a_rotation_sensor"></a>

## `the_angle_sensor_role_is_not_a_rotation_sensor`

```cpp
inline void the_angle_sensor_role_is_not_a_rotation_sensor()
```

Refusal::TheAngleSensorRoleIsNotARotationSensor's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:728`](../../include/shulib/mechanism/spec.hpp#L728).*

<a id="an_angle_sensor_role_on_a_mechanism_that_is_not_position_or_speed"></a>

## `an_angle_sensor_role_on_a_mechanism_that_is_not_position_or_speed`

```cpp
inline void an_angle_sensor_role_on_a_mechanism_that_is_not_position_or_speed()
```

Refusal::AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:730`](../../include/shulib/mechanism/spec.hpp#L730).*

<a id="the_home_switch_role_is_not_a_limit_switch"></a>

## `the_home_switch_role_is_not_a_limit_switch`

```cpp
inline void the_home_switch_role_is_not_a_limit_switch()
```

Refusal::TheHomeSwitchRoleIsNotALimitSwitch's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:732`](../../include/shulib/mechanism/spec.hpp#L732).*

<a id="a_home_switch_role_on_a_mechanism_that_is_not_position"></a>

## `a_home_switch_role_on_a_mechanism_that_is_not_position`

```cpp
inline void a_home_switch_role_on_a_mechanism_that_is_not_position()
```

Refusal::AHomeSwitchRoleOnAMechanismThatIsNotPosition's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:734`](../../include/shulib/mechanism/spec.hpp#L734).*

<a id="a_piece_sensor_cannot_be_a_rotation_sensor"></a>

## `a_piece_sensor_cannot_be_a_rotation_sensor`

```cpp
inline void a_piece_sensor_cannot_be_a_rotation_sensor()
```

Refusal::APieceSensorCannotBeARotationSensor's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:736`](../../include/shulib/mechanism/spec.hpp#L736).*

<a id="a_distance_piece_sensor_needs_its_sees_within_distance_more_than_zero"></a>

## `a_distance_piece_sensor_needs_its_sees_within_distance_more_than_zero`

```cpp
inline void a_distance_piece_sensor_needs_its_sees_within_distance_more_than_zero()
```

Refusal::ADistancePieceSensorNeedsItsSeesWithinDistance0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:738`](../../include/shulib/mechanism/spec.hpp#L738).*

<a id="an_optical_piece_sensor_needs_its_proximity_threshold_in_above_0_and_at_most_1"></a>

## `an_optical_piece_sensor_needs_its_proximity_threshold_in_above_0_and_at_most_1`

```cpp
inline void an_optical_piece_sensor_needs_its_proximity_threshold_in_above_0_and_at_most_1()
```

Refusal::AnOpticalPieceSensorNeedsItsProximityThresholdIn01's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:740`](../../include/shulib/mechanism/spec.hpp#L740).*

<a id="power_maxvolts_is_not_in_above_0_and_at_most_12"></a>

## `power_maxvolts_is_not_in_above_0_and_at_most_12`

```cpp
inline void power_maxvolts_is_not_in_above_0_and_at_most_12()
```

Refusal::PowerMaxVoltsIsNotIn012's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:742`](../../include/shulib/mechanism/spec.hpp#L742).*

<a id="position_units_unset"></a>

## `position_units_unset`

```cpp
inline void position_units_unset()
```

Refusal::PositionUnitsUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:744`](../../include/shulib/mechanism/spec.hpp#L744).*

<a id="position_gearratio_must_be_more_than_zero"></a>

## `position_gearratio_must_be_more_than_zero`

```cpp
inline void position_gearratio_must_be_more_than_zero()
```

Refusal::PositionGearRatioMustBe0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:746`](../../include/shulib/mechanism/spec.hpp#L746).*

<a id="position_a_rotary_mechanism_is_in_degrees"></a>

## `position_a_rotary_mechanism_is_in_degrees`

```cpp
inline void position_a_rotary_mechanism_is_in_degrees()
```

Refusal::PositionARotaryMechanismIsInDegrees's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:748`](../../include/shulib/mechanism/spec.hpp#L748).*

<a id="position_a_lift_is_in_inches"></a>

## `position_a_lift_is_in_inches`

```cpp
inline void position_a_lift_is_in_inches()
```

Refusal::PositionALiftIsInInches's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:750`](../../include/shulib/mechanism/spec.hpp#L750).*

<a id="position_spool_diameter_must_be_more_than_zero"></a>

## `position_spool_diameter_must_be_more_than_zero`

```cpp
inline void position_spool_diameter_must_be_more_than_zero()
```

Refusal::PositionSpoolDiameterMustBe0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:752`](../../include/shulib/mechanism/spec.hpp#L752).*

<a id="position_arm_length_must_be_more_than_zero"></a>

## `position_arm_length_must_be_more_than_zero`

```cpp
inline void position_arm_length_must_be_more_than_zero()
```

Refusal::PositionArmLengthMustBe0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:754`](../../include/shulib/mechanism/spec.hpp#L754).*

<a id="position_an_arm_needs_one_to_three_stages"></a>

## `position_an_arm_needs_one_to_three_stages`

```cpp
inline void position_an_arm_needs_one_to_three_stages()
```

Refusal::PositionAnArmNeeds13Stages's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:756`](../../include/shulib/mechanism/spec.hpp#L756).*

<a id="position_an_arms_travel_limits_reach_past_its_linkage_the_height_formula_has_no_inverse_there"></a>

## `position_an_arms_travel_limits_reach_past_its_linkage_the_height_formula_has_no_inverse_there`

```cpp
inline void position_an_arms_travel_limits_reach_past_its_linkage_the_height_formula_has_no_inverse_there()
```

Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:758`](../../include/shulib/mechanism/spec.hpp#L758).*

<a id="position_a_lookup_table_needs_two_to_eight_points"></a>

## `position_a_lookup_table_needs_two_to_eight_points`

```cpp
inline void position_a_lookup_table_needs_two_to_eight_points()
```

Refusal::PositionALookupTableNeeds28Points's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:760`](../../include/shulib/mechanism/spec.hpp#L760).*

<a id="position_the_lookup_table_is_not_strictly_increasing"></a>

## `position_the_lookup_table_is_not_strictly_increasing`

```cpp
inline void position_the_lookup_table_is_not_strictly_increasing()
```

Refusal::PositionTheLookupTableIsNotStrictlyIncreasing's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:762`](../../include/shulib/mechanism/spec.hpp#L762).*

<a id="position_travel_limits_missing_or_min_at_least_max"></a>

## `position_travel_limits_missing_or_min_at_least_max`

```cpp
inline void position_travel_limits_missing_or_min_at_least_max()
```

Refusal::PositionTravelLimitsMissingOrMinMax's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:764`](../../include/shulib/mechanism/spec.hpp#L764).*

<a id="position_the_profile_needs_maxvelocity_and_maxacceleration_more_than_zero"></a>

## `position_the_profile_needs_maxvelocity_and_maxacceleration_more_than_zero`

```cpp
inline void position_the_profile_needs_maxvelocity_and_maxacceleration_more_than_zero()
```

Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:766`](../../include/shulib/mechanism/spec.hpp#L766).*

<a id="position_tolerance_must_be_more_than_zero"></a>

## `position_tolerance_must_be_more_than_zero`

```cpp
inline void position_tolerance_must_be_more_than_zero()
```

Refusal::PositionToleranceMustBe0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:768`](../../include/shulib/mechanism/spec.hpp#L768).*

<a id="position_gravity_unset"></a>

## `position_gravity_unset`

```cpp
inline void position_gravity_unset()
```

Refusal::PositionGravityUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:770`](../../include/shulib/mechanism/spec.hpp#L770).*

<a id="position_homing_method_unset"></a>

## `position_homing_method_unset`

```cpp
inline void position_homing_method_unset()
```

Refusal::PositionHomingMethodUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:772`](../../include/shulib/mechanism/spec.hpp#L772).*

<a id="position_homing_by_switch_needs_a_homeswitch_role"></a>

## `position_homing_by_switch_needs_a_homeswitch_role`

```cpp
inline void position_homing_by_switch_needs_a_homeswitch_role()
```

Refusal::PositionHomingBySwitchNeedsAHomeSwitchRole's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:774`](../../include/shulib/mechanism/spec.hpp#L774).*

<a id="position_absolute_homing_needs_an_anglesensor_rotation_role"></a>

## `position_absolute_homing_needs_an_anglesensor_rotation_role`

```cpp
inline void position_absolute_homing_needs_an_anglesensor_rotation_role()
```

Refusal::PositionAbsoluteHomingNeedsAnAngleSensorRotationRole's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:776`](../../include/shulib/mechanism/spec.hpp#L776).*

<a id="position_homing_needs_homevolts"></a>

## `position_homing_needs_homevolts`

```cpp
inline void position_homing_needs_homevolts()
```

Refusal::PositionHomingNeedsHomeVolts's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:778`](../../include/shulib/mechanism/spec.hpp#L778).*

<a id="position_stall_homing_needs_its_stall_signature"></a>

## `position_stall_homing_needs_its_stall_signature`

```cpp
inline void position_stall_homing_needs_its_stall_signature()
```

Refusal::PositionStallHomingNeedsItsStallSignature's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:780`](../../include/shulib/mechanism/spec.hpp#L780).*

<a id="position_the_home_position_is_outside_the_travel_limits"></a>

## `position_the_home_position_is_outside_the_travel_limits`

```cpp
inline void position_the_home_position_is_outside_the_travel_limits()
```

Refusal::PositionTheHomePositionIsOutsideTheTravelLimits's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:782`](../../include/shulib/mechanism/spec.hpp#L782).*

<a id="position_more_presets_than_fit"></a>

## `position_more_presets_than_fit`

```cpp
inline void position_more_presets_than_fit()
```

Refusal::PositionMorePresetsThanFit's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:784`](../../include/shulib/mechanism/spec.hpp#L784).*

<a id="position_a_preset_has_no_name"></a>

## `position_a_preset_has_no_name`

```cpp
inline void position_a_preset_has_no_name()
```

Refusal::PositionAPresetHasNoName's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:786`](../../include/shulib/mechanism/spec.hpp#L786).*

<a id="position_a_preset_is_outside_the_travel_limits"></a>

## `position_a_preset_is_outside_the_travel_limits`

```cpp
inline void position_a_preset_is_outside_the_travel_limits()
```

Refusal::PositionAPresetIsOutsideTheTravelLimits's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:788`](../../include/shulib/mechanism/spec.hpp#L788).*

<a id="speed_gearratio_and_its_free_speed_must_be_more_than_zero"></a>

## `speed_gearratio_and_its_free_speed_must_be_more_than_zero`

```cpp
inline void speed_gearratio_and_its_free_speed_must_be_more_than_zero()
```

Refusal::SpeedGearRatioAndMaxRpmMustBe0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:790`](../../include/shulib/mechanism/spec.hpp#L790).*

<a id="speed_the_at_speed_band_must_be_more_than_zero"></a>

## `speed_the_at_speed_band_must_be_more_than_zero`

```cpp
inline void speed_the_at_speed_band_must_be_more_than_zero()
```

Refusal::SpeedTheAtSpeedBandMustBe0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:792`](../../include/shulib/mechanism/spec.hpp#L792).*

<a id="speed_more_presets_than_fit"></a>

## `speed_more_presets_than_fit`

```cpp
inline void speed_more_presets_than_fit()
```

Refusal::SpeedMorePresetsThanFit's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:794`](../../include/shulib/mechanism/spec.hpp#L794).*

<a id="speed_a_preset_has_no_name"></a>

## `speed_a_preset_has_no_name`

```cpp
inline void speed_a_preset_has_no_name()
```

Refusal::SpeedAPresetHasNoName's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:796`](../../include/shulib/mechanism/spec.hpp#L796).*

<a id="speed_a_preset_is_past_its_free_speed"></a>

## `speed_a_preset_is_past_its_free_speed`

```cpp
inline void speed_a_preset_is_past_its_free_speed()
```

Refusal::SpeedAPresetIsPastMaxRpm's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:798`](../../include/shulib/mechanism/spec.hpp#L798).*

<a id="pneumatic_two_or_three_positions"></a>

## `pneumatic_two_or_three_positions`

```cpp
inline void pneumatic_two_or_three_positions()
```

Refusal::Pneumatic2Or3Positions's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:800`](../../include/shulib/mechanism/spec.hpp#L800).*

<a id="pneumatic_start_or_safe_is_not_one_of_its_positions"></a>

## `pneumatic_start_or_safe_is_not_one_of_its_positions`

```cpp
inline void pneumatic_start_or_safe_is_not_one_of_its_positions()
```

Refusal::PneumaticStartSafeIsNotOneOfItsPositions's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:802`](../../include/shulib/mechanism/spec.hpp#L802).*

<a id="pneumatic_actuationtime_must_be_more_than_zero"></a>

## `pneumatic_actuationtime_must_be_more_than_zero`

```cpp
inline void pneumatic_actuationtime_must_be_more_than_zero()
```

Refusal::PneumaticActuationTimeMustBe0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:804`](../../include/shulib/mechanism/spec.hpp#L804).*

<a id="pneumatic_a_position_has_no_name"></a>

## `pneumatic_a_position_has_no_name`

```cpp
inline void pneumatic_a_position_has_no_name()
```

Refusal::PneumaticAPositionHasNoName's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:806`](../../include/shulib/mechanism/spec.hpp#L806).*

<a id="pneumatic_two_positions_have_the_same_line_levels"></a>

## `pneumatic_two_positions_have_the_same_line_levels`

```cpp
inline void pneumatic_two_positions_have_the_same_line_levels()
```

Refusal::PneumaticTwoPositionsHaveTheSameLineLevels's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:808`](../../include/shulib/mechanism/spec.hpp#L808).*

<a id="pneumatic_its_start_names_none_of_its_positions"></a>

## `pneumatic_its_start_names_none_of_its_positions`

```cpp
inline void pneumatic_its_start_names_none_of_its_positions()
```

Refusal::PneumaticItsStartNamesNoneOfItsPositions's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:810`](../../include/shulib/mechanism/spec.hpp#L810).*

<a id="pneumatic_its_safe_names_none_of_its_positions"></a>

## `pneumatic_its_safe_names_none_of_its_positions`

```cpp
inline void pneumatic_its_safe_names_none_of_its_positions()
```

Refusal::PneumaticItsSafeNamesNoneOfItsPositions's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:812`](../../include/shulib/mechanism/spec.hpp#L812).*

<a id="a_preset_or_position_name_does_not_fit_fifteen_characters"></a>

## `a_preset_or_position_name_does_not_fit_fifteen_characters`

```cpp
inline void a_preset_or_position_name_does_not_fit_fifteen_characters()
```

Refusal::APresetOrPositionNameDoesNotFit15Characters's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:814`](../../include/shulib/mechanism/spec.hpp#L814).*

<a id="two_presets_or_positions_share_a_name"></a>

## `two_presets_or_positions_share_a_name`

```cpp
inline void two_presets_or_positions_share_a_name()
```

Refusal::TwoPresetsOrPositionsShareAName's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:816`](../../include/shulib/mechanism/spec.hpp#L816).*

<a id="absolute_homing_the_sensors_reading_at_zero_must_be_in_from_0_up_to_360"></a>

## `absolute_homing_the_sensors_reading_at_zero_must_be_in_from_0_up_to_360`

```cpp
inline void absolute_homing_the_sensors_reading_at_zero_must_be_in_from_0_up_to_360()
```

Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:818`](../../include/shulib/mechanism/spec.hpp#L818).*

<a id="position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current"></a>

## `position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current`

```cpp
inline void position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current()
```

Refusal::PositionTheStallSignatureIsUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:820`](../../include/shulib/mechanism/spec.hpp#L820).*

<a id="position_the_homing_push_points_away_from_its_home_end"></a>

## `position_the_homing_push_points_away_from_its_home_end`

```cpp
inline void position_the_homing_push_points_away_from_its_home_end()
```

Refusal::PositionHomingPushesAwayFromItsHomeEnd's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:822`](../../include/shulib/mechanism/spec.hpp#L822).*

<a id="speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current"></a>

## `speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current`

```cpp
inline void speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current()
```

Refusal::SpeedTheStallSignatureIsUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:824`](../../include/shulib/mechanism/spec.hpp#L824).*

<a id="a_stall_signatures_amps_must_be_below_the_motors_2_5_a_current_limit"></a>

## `a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_limit`

```cpp
inline void a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_limit()
```

Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:826`](../../include/shulib/mechanism/spec.hpp#L826).*

<a id="a_stall_signature_needs_belowmotorrpm_above_0"></a>

## `a_stall_signature_needs_belowMotorRpm_above_0`

```cpp
inline void a_stall_signature_needs_belowMotorRpm_above_0()
```

Refusal::AStallSignatureNeedsBelowMotorRpmAbove0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:828`](../../include/shulib/mechanism/spec.hpp#L828).*

<a id="a_stall_signature_needs_lasting_above_0"></a>

## `a_stall_signature_needs_lasting_above_0`

```cpp
inline void a_stall_signature_needs_lasting_above_0()
```

Refusal::AStallSignatureNeedsLastingAbove0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:830`](../../include/shulib/mechanism/spec.hpp#L830).*

<a id="a_stall_signature_needs_its_amps_above_0"></a>

## `a_stall_signature_needs_its_amps_above_0`

```cpp
inline void a_stall_signature_needs_its_amps_above_0()
```

Refusal::AStallSignatureNeedsAmpsAbove0's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:832`](../../include/shulib/mechanism/spec.hpp#L832).*

<a id="position_a_lifts_shape_is_unset_give_it_spool_linkage_or_measured"></a>

## `position_a_lifts_shape_is_unset_give_it_spool_linkage_or_measured`

```cpp
inline void position_a_lifts_shape_is_unset_give_it_spool_linkage_or_measured()
```

Refusal::PositionGeometryUNSET's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:834`](../../include/shulib/mechanism/spec.hpp#L834).*

<a id="position_the_settle_time_must_not_be_negative"></a>

## `position_the_settle_time_must_not_be_negative`

```cpp
inline void position_the_settle_time_must_not_be_negative()
```

Refusal::PositionTheSettleTimeIsNegative's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:836`](../../include/shulib/mechanism/spec.hpp#L836).*

<a id="speed_the_at_speed_lasting_must_not_be_negative"></a>

## `speed_the_at_speed_lasting_must_not_be_negative`

```cpp
inline void speed_the_at_speed_lasting_must_not_be_negative()
```

Refusal::SpeedTheAtSpeedLastingIsNegative's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:838`](../../include/shulib/mechanism/spec.hpp#L838).*

<a id="pneumatic_confirmwithin_must_not_be_negative"></a>

## `pneumatic_confirmwithin_must_not_be_negative`

```cpp
inline void pneumatic_confirmwithin_must_not_be_negative()
```

Refusal::PneumaticTheConfirmWindowIsNegative's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:840`](../../include/shulib/mechanism/spec.hpp#L840).*

<a id="position_the_homing_push_must_be_within_12_volts_either_way"></a>

## `position_the_homing_push_must_be_within_12_volts_either_way`

```cpp
inline void position_the_homing_push_must_be_within_12_volts_either_way()
```

Refusal::PositionTheHomingPushIsPast12Volts's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:842`](../../include/shulib/mechanism/spec.hpp#L842).*

<a id="position_the_gravity_hold_must_be_within_12_volts_either_way"></a>

## `position_the_gravity_hold_must_be_within_12_volts_either_way`

```cpp
inline void position_the_gravity_hold_must_be_within_12_volts_either_way()
```

Refusal::PositionTheGravityHoldIsPast12Volts's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:844`](../../include/shulib/mechanism/spec.hpp#L844).*

<a id="position_unhomedvolts_must_be_from_0_to_12"></a>

## `position_unhomedvolts_must_be_from_0_to_12`

```cpp
inline void position_unhomedvolts_must_be_from_0_to_12()
```

Refusal::PositionUnhomedVoltsIsNotIn0To12's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:846`](../../include/shulib/mechanism/spec.hpp#L846).*

<a id="position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor"></a>

## `position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor`

```cpp
inline void position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor()
```

Refusal::PositionAbsoluteHomingTravelSpansAWholeSensorTurn's name at the robot file's line.

*free function, declared at [`include/shulib/mechanism/spec.hpp:848`](../../include/shulib/mechanism/spec.hpp#L848).*

<a id="refuse"></a>

## `refuse`

```cpp
consteval void refuse(Refusal r)
```

Stop the build with `r`'s named function (consteval contexts only); None does nothing.

*free function, declared at [`include/shulib/mechanism/spec.hpp:852`](../../include/shulib/mechanism/spec.hpp#L852).*

<a id="specrefusal"></a>

## `specRefusal`

```cpp
[[nodiscard]] constexpr Refusal specRefusal(const MechanismSpec& s) noexcept
```

THE BUILD CHECK (header): the FIRST thing wrong with `s`, or Refusal::None. constexpr, so a robot file's list is checked at build (wire() calls refuse(specRefusal(spec))) and a host test runs the same function at run time. The ORDER is pinned by a test, one property per reason.

*free function, declared at [`include/shulib/mechanism/spec.hpp:1108`](../../include/shulib/mechanism/spec.hpp#L1108).*

<a id="specproblem"></a>

## `specProblem`

```cpp
[[nodiscard]] constexpr const char* specProblem(const MechanismSpec& s) noexcept
```

The same check as a SENTENCE (nullptr = valid) — run-time callers, host tests, G3a's agreement test.

*free function, declared at [`include/shulib/mechanism/spec.hpp:1427`](../../include/shulib/mechanism/spec.hpp#L1427).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 45 lines</summary>

```text

 mechanism/spec.hpp — ONE MECHANISM AS DATA, and THE BUILD CHECK (the mechanism core, chunk
 MECH-A1, 2026-10-08).

 ── What this file is ───────────────────────────────────────────────────────────────
 The device-independent form every mechanism declaration becomes (mechanism/kinds.hpp's
 `spec()`): what a mechanism IS — its kind, its ROLES (motors, an angle sensor, a home switch, a
 piece sensor, 3-wire lines) and its numbers — and never which PORT. The ports are the wiring
 (mechanism/wiring.hpp), a separate block of the robot file: re-cabling changes only that block
 (the team lead, 2026-09-28: "mechanisms should be port agnostic ... bound to ports after they are
 created"). The mechanism service, the Robot's projection, the direction check, the route ids
 (mechanism/route_ids.hpp) and the robot panel's agreement test (G3a) all read THIS form.

 Every type here is a constexpr aggregate with fixed-width fields and bounded arrays — no
 allocation, no pointer the build cannot see through except the recorded file names — so the
 robot file's generator (G3a) can emit it and the COMPILER can check it.

 ── EVERY FIELD HAS AN UNSET VALUE, and nothing the team owns is invented (P6) ──────
 A field nobody stated is the byte 0 (or 0.0) and is REFUSED by specRefusal(), naming it. The
 three kinds of number that are NOT the team's — the controllers' starting gains, the smooth
 move's starting speed and acceleration, and the "reached" tolerance — are the library's STARTING
 values (kinds.hpp's positionStart(); PROVISIONAL: HA-182, registered by MECH-A3 with the behaviours
 that run them, mechanism/service.hpp).

 ── THE BUILD CHECK, three ways, one list ───────────────────────────────────────────
 specRefusal() is THE check: the FIRST thing wrong with one spec, as a `Refusal`, checked in a
 fixed order (one property per reason, so a test can pin the order). Each reason has:
   * a SENTENCE (refusalText) — for a run-time caller, a host test, and the robot panel's
     validator, which mirrors this list by sentence (G3a; its agreement test proves it);
   * a NAMED FUNCTION in namespace `refused` that is deliberately NOT constexpr — refuse() calls
     it from a consteval context, so the build stops and GCC prints the reason as the function's
     name: "call to non-'constexpr' function 'void shulib::mechanism::refused::
     position_a_preset_is_outside_the_travel_limits()'". The robot file's line is GCC's first
     "in 'constexpr' expansion of" note (the guide's "reading a refusal").
 Appended only, like every enum the robot file and the panel share: a new reason goes at the END,
 so G3a's mirror never renumbers.

 ── Units ───────────────────────────────────────────────────────────────────────────
 A position mechanism is in DEGREES of its output (an arm) or INCHES (a lift: a spool, a linkage, a
 measured table). EVERY GAIN is per OUTPUT-SHAFT DEGREE whatever the units — the loop runs on the
 output angle, where gravity's cosine lives; the limits, presets, tolerance and profile stay in
 the mechanism's units (angleDegToPosition / positionToAngleDeg convert). A speed mechanism is in
 RPM of its output.

 PROS-free, sim-free, allocation-free.
```

</details>
