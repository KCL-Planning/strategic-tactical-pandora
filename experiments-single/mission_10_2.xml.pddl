(define (problem pandora_strategic_mission)
(:domain pandora_domain_strategic)
(:objects
auv - vehicle
mission_site_start_point_0 mission_site_start_point_1 wp_auv0 - waypoint
Mission0 Mission1 Mission10 Mission100 Mission101 Mission102 Mission103 Mission104 Mission105 Mission12 Mission13 Mission14 Mission15 Mission16 Mission17 Mission18 Mission19 Mission21 Mission22 Mission23 Mission24 Mission25 Mission26 Mission27 Mission28 Mission3 Mission30 Mission31 Mission32 Mission33 Mission34 Mission35 Mission36 Mission37 Mission39 Mission4 Mission40 Mission41 Mission42 Mission43 Mission44 Mission45 Mission46 Mission48 Mission49 Mission5 Mission50 Mission51 Mission52 Mission53 Mission54 Mission55 Mission57 Mission58 Mission59 Mission6 Mission60 Mission61 Mission62 Mission63 Mission64 Mission65 Mission66 Mission67 Mission68 Mission69 Mission7 Mission70 Mission71 Mission72 Mission73 Mission74 Mission75 Mission76 Mission77 Mission78 Mission79 Mission8 Mission80 Mission82 Mission83 Mission84 Mission85 Mission86 Mission87 Mission88 Mission89 Mission9 Mission90 Mission91 Mission92 Mission93 Mission94 Mission95 Mission96 Mission97 Mission98 Mission99 - mission
)
(:init
(vehicle_free auv)
(= (mission_total) 0)
(at auv wp_auv0) (= (charge auv) 1200)

(recharge_at mission_site_start_point_0)

(active Mission0)
(active Mission1)
(active Mission10)
(active Mission100)
(active Mission101)
(active Mission102)
(active Mission103)
(active Mission104)
(active Mission105)
(active Mission12)
(active Mission13)
(active Mission14)
(active Mission15)
(active Mission16)
(active Mission17)
(active Mission18)
(active Mission19)
(active Mission21)
(active Mission22)
(active Mission23)
(active Mission24)
(active Mission25)
(active Mission26)
(active Mission27)
(active Mission28)
(active Mission3)
(active Mission30)
(active Mission31)
(active Mission32)
(active Mission33)
(active Mission34)
(active Mission35)
(active Mission36)
(active Mission37)
(active Mission39)
(active Mission4)
(active Mission40)
(active Mission41)
(active Mission42)
(active Mission43)
(active Mission44)
(active Mission45)
(active Mission46)
(active Mission48)
(active Mission49)
(active Mission5)
(active Mission50)
(active Mission51)
(active Mission52)
(active Mission53)
(active Mission54)
(active Mission55)
(active Mission57)
(active Mission58)
(active Mission59)
(active Mission6)
(active Mission60)
(active Mission61)
(active Mission62)
(active Mission63)
(active Mission64)
(active Mission65)
(active Mission66)
(active Mission67)
(active Mission68)
(active Mission69)
(active Mission7)
(active Mission70)
(active Mission71)
(active Mission72)
(active Mission73)
(active Mission74)
(active Mission75)
(active Mission76)
(active Mission77)
(active Mission78)
(active Mission79)
(active Mission8)
(active Mission80)
(active Mission82)
(active Mission83)
(active Mission84)
(active Mission85)
(active Mission86)
(active Mission87)
(active Mission88)
(active Mission89)
(active Mission9)
(active Mission90)
(active Mission91)
(active Mission92)
(active Mission93)
(active Mission94)
(active Mission95)
(active Mission96)
(active Mission97)
(active Mission98)
(active Mission99)

(at 4100 (not (active Mission0)))
(at 7100 (not (active Mission1)))
(at 86400 (not (active Mission10)))
(at 86400 (not (active Mission100)))
(at 86400 (not (active Mission101)))
(at 86400 (not (active Mission102)))
(at 86400 (not (active Mission103)))
(at 86400 (not (active Mission104)))
(at 86400 (not (active Mission105)))
(at 86400 (not (active Mission12)))
(at 86400 (not (active Mission13)))
(at 86400 (not (active Mission14)))
(at 86400 (not (active Mission15)))
(at 86400 (not (active Mission16)))
(at 86400 (not (active Mission17)))
(at 86400 (not (active Mission18)))
(at 86400 (not (active Mission19)))
(at 86400 (not (active Mission21)))
(at 86400 (not (active Mission22)))
(at 86400 (not (active Mission23)))
(at 86400 (not (active Mission24)))
(at 86400 (not (active Mission25)))
(at 86400 (not (active Mission26)))
(at 86400 (not (active Mission27)))
(at 86400 (not (active Mission28)))
(at 86400 (not (active Mission3)))
(at 86400 (not (active Mission30)))
(at 86400 (not (active Mission31)))
(at 86400 (not (active Mission32)))
(at 86400 (not (active Mission33)))
(at 86400 (not (active Mission34)))
(at 86400 (not (active Mission35)))
(at 86400 (not (active Mission36)))
(at 86400 (not (active Mission37)))
(at 86400 (not (active Mission39)))
(at 86400 (not (active Mission4)))
(at 86400 (not (active Mission40)))
(at 86400 (not (active Mission41)))
(at 86400 (not (active Mission42)))
(at 86400 (not (active Mission43)))
(at 86400 (not (active Mission44)))
(at 86400 (not (active Mission45)))
(at 86400 (not (active Mission46)))
(at 86400 (not (active Mission48)))
(at 86400 (not (active Mission49)))
(at 86400 (not (active Mission5)))
(at 86400 (not (active Mission50)))
(at 86400 (not (active Mission51)))
(at 86400 (not (active Mission52)))
(at 86400 (not (active Mission53)))
(at 86400 (not (active Mission54)))
(at 86400 (not (active Mission55)))
(at 86400 (not (active Mission57)))
(at 86400 (not (active Mission58)))
(at 86400 (not (active Mission59)))
(at 86400 (not (active Mission6)))
(at 86400 (not (active Mission60)))
(at 86400 (not (active Mission61)))
(at 86400 (not (active Mission62)))
(at 86400 (not (active Mission63)))
(at 86400 (not (active Mission64)))
(at 86400 (not (active Mission65)))
(at 86400 (not (active Mission66)))
(at 86400 (not (active Mission67)))
(at 86400 (not (active Mission68)))
(at 86400 (not (active Mission69)))
(at 86400 (not (active Mission7)))
(at 86400 (not (active Mission70)))
(at 86400 (not (active Mission71)))
(at 86400 (not (active Mission72)))
(at 86400 (not (active Mission73)))
(at 86400 (not (active Mission74)))
(at 86400 (not (active Mission75)))
(at 86400 (not (active Mission76)))
(at 86400 (not (active Mission77)))
(at 86400 (not (active Mission78)))
(at 86400 (not (active Mission79)))
(at 86400 (not (active Mission8)))
(at 86400 (not (active Mission80)))
(at 86400 (not (active Mission82)))
(at 86400 (not (active Mission83)))
(at 86400 (not (active Mission84)))
(at 86400 (not (active Mission85)))
(at 86400 (not (active Mission86)))
(at 86400 (not (active Mission87)))
(at 86400 (not (active Mission88)))
(at 86400 (not (active Mission89)))
(at 86400 (not (active Mission9)))
(at 86400 (not (active Mission90)))
(at 86400 (not (active Mission91)))
(at 86400 (not (active Mission92)))
(at 86400 (not (active Mission93)))
(at 86400 (not (active Mission94)))
(at 86400 (not (active Mission95)))
(at 86400 (not (active Mission96)))
(at 86400 (not (active Mission97)))
(at 86400 (not (active Mission98)))
(at 86400 (not (active Mission99)))

(in Mission0 mission_site_start_point_1)
(in Mission1 mission_site_start_point_1)
(in Mission10 mission_site_start_point_1)
(in Mission100 mission_site_start_point_1)
(in Mission101 mission_site_start_point_1)
(in Mission102 mission_site_start_point_1)
(in Mission103 mission_site_start_point_1)
(in Mission104 mission_site_start_point_1)
(in Mission105 mission_site_start_point_1)
(in Mission12 mission_site_start_point_1)
(in Mission13 mission_site_start_point_1)
(in Mission14 mission_site_start_point_1)
(in Mission15 mission_site_start_point_1)
(in Mission16 mission_site_start_point_1)
(in Mission17 mission_site_start_point_1)
(in Mission18 mission_site_start_point_1)
(in Mission19 mission_site_start_point_1)
(in Mission21 mission_site_start_point_1)
(in Mission22 mission_site_start_point_1)
(in Mission23 mission_site_start_point_1)
(in Mission24 mission_site_start_point_1)
(in Mission25 mission_site_start_point_1)
(in Mission26 mission_site_start_point_1)
(in Mission27 mission_site_start_point_1)
(in Mission28 mission_site_start_point_1)
(in Mission3 mission_site_start_point_1)
(in Mission30 mission_site_start_point_1)
(in Mission31 mission_site_start_point_1)
(in Mission32 mission_site_start_point_1)
(in Mission33 mission_site_start_point_1)
(in Mission34 mission_site_start_point_1)
(in Mission35 mission_site_start_point_1)
(in Mission36 mission_site_start_point_1)
(in Mission37 mission_site_start_point_1)
(in Mission39 mission_site_start_point_1)
(in Mission4 mission_site_start_point_1)
(in Mission40 mission_site_start_point_1)
(in Mission41 mission_site_start_point_1)
(in Mission42 mission_site_start_point_1)
(in Mission43 mission_site_start_point_1)
(in Mission44 mission_site_start_point_1)
(in Mission45 mission_site_start_point_1)
(in Mission46 mission_site_start_point_1)
(in Mission48 mission_site_start_point_1)
(in Mission49 mission_site_start_point_1)
(in Mission5 mission_site_start_point_1)
(in Mission50 mission_site_start_point_1)
(in Mission51 mission_site_start_point_1)
(in Mission52 mission_site_start_point_1)
(in Mission53 mission_site_start_point_1)
(in Mission54 mission_site_start_point_1)
(in Mission55 mission_site_start_point_1)
(in Mission57 mission_site_start_point_1)
(in Mission58 mission_site_start_point_1)
(in Mission59 mission_site_start_point_1)
(in Mission6 mission_site_start_point_1)
(in Mission60 mission_site_start_point_1)
(in Mission61 mission_site_start_point_1)
(in Mission62 mission_site_start_point_1)
(in Mission63 mission_site_start_point_1)
(in Mission64 mission_site_start_point_1)
(in Mission65 mission_site_start_point_1)
(in Mission66 mission_site_start_point_1)
(in Mission67 mission_site_start_point_1)
(in Mission68 mission_site_start_point_1)
(in Mission69 mission_site_start_point_1)
(in Mission7 mission_site_start_point_1)
(in Mission70 mission_site_start_point_1)
(in Mission71 mission_site_start_point_1)
(in Mission72 mission_site_start_point_1)
(in Mission73 mission_site_start_point_1)
(in Mission74 mission_site_start_point_1)
(in Mission75 mission_site_start_point_1)
(in Mission76 mission_site_start_point_1)
(in Mission77 mission_site_start_point_1)
(in Mission78 mission_site_start_point_1)
(in Mission79 mission_site_start_point_1)
(in Mission8 mission_site_start_point_1)
(in Mission80 mission_site_start_point_1)
(in Mission82 mission_site_start_point_1)
(in Mission83 mission_site_start_point_1)
(in Mission84 mission_site_start_point_1)
(in Mission85 mission_site_start_point_1)
(in Mission86 mission_site_start_point_1)
(in Mission87 mission_site_start_point_1)
(in Mission88 mission_site_start_point_1)
(in Mission89 mission_site_start_point_1)
(in Mission9 mission_site_start_point_1)
(in Mission90 mission_site_start_point_1)
(in Mission91 mission_site_start_point_1)
(in Mission92 mission_site_start_point_1)
(in Mission93 mission_site_start_point_1)
(in Mission94 mission_site_start_point_1)
(in Mission95 mission_site_start_point_1)
(in Mission96 mission_site_start_point_1)
(in Mission97 mission_site_start_point_1)
(in Mission98 mission_site_start_point_1)
(in Mission99 mission_site_start_point_1)

(= (mission_duration Mission0) 261.867)
(= (mission_duration Mission1) 242.066)
(= (mission_duration Mission10) 117.739)
(= (mission_duration Mission100) 107.368)
(= (mission_duration Mission101) 93.333)
(= (mission_duration Mission102) 102.021)
(= (mission_duration Mission103) 114.632)
(= (mission_duration Mission104) 106.106)
(= (mission_duration Mission105) 91.787)
(= (mission_duration Mission12) 154.668)
(= (mission_duration Mission13) 157.892)
(= (mission_duration Mission14) 151.502)
(= (mission_duration Mission15) 135.29)
(= (mission_duration Mission16) 139.066)
(= (mission_duration Mission17) 142.864)
(= (mission_duration Mission18) 127.896)
(= (mission_duration Mission19) 163.09)
(= (mission_duration Mission21) 139.066)
(= (mission_duration Mission22) 142.864)
(= (mission_duration Mission23) 135.29)
(= (mission_duration Mission24) 151.502)
(= (mission_duration Mission25) 154.668)
(= (mission_duration Mission26) 157.892)
(= (mission_duration Mission27) 100.141)
(= (mission_duration Mission28) 163.824)
(= (mission_duration Mission3) 142.946)
(= (mission_duration Mission30) 135.768)
(= (mission_duration Mission31) 132.024)
(= (mission_duration Mission32) 139.531)
(= (mission_duration Mission33) 154.735)
(= (mission_duration Mission34) 151.457)
(= (mission_duration Mission35) 148.225)
(= (mission_duration Mission36) 160.777)
(= (mission_duration Mission37) 125.803)
(= (mission_duration Mission39) 139.066)
(= (mission_duration Mission4) 139.487)
(= (mission_duration Mission40) 142.864)
(= (mission_duration Mission41) 135.29)
(= (mission_duration Mission42) 151.502)
(= (mission_duration Mission43) 154.668)
(= (mission_duration Mission44) 157.892)
(= (mission_duration Mission45) 100.141)
(= (mission_duration Mission46) 163.824)
(= (mission_duration Mission48) 135.768)
(= (mission_duration Mission49) 132.024)
(= (mission_duration Mission5) 146.439)
(= (mission_duration Mission50) 139.531)
(= (mission_duration Mission51) 154.735)
(= (mission_duration Mission52) 151.457)
(= (mission_duration Mission53) 148.225)
(= (mission_duration Mission54) 160.777)
(= (mission_duration Mission55) 125.803)
(= (mission_duration Mission57) 104.821)
(= (mission_duration Mission58) 97.367)
(= (mission_duration Mission59) 112.248)
(= (mission_duration Mission6) 136.128)
(= (mission_duration Mission60) 118.577)
(= (mission_duration Mission61) 103.451)
(= (mission_duration Mission62) 95.881)
(= (mission_duration Mission63) 111.04)
(= (mission_duration Mission64) 117.438)
(= (mission_duration Mission65) 160.024)
(= (mission_duration Mission66) 164.312)
(= (mission_duration Mission67) 180.286)
(= (mission_duration Mission68) 176.439)
(= (mission_duration Mission69) 159.274)
(= (mission_duration Mission7) 132.262)
(= (mission_duration Mission70) 163.586)
(= (mission_duration Mission71) 179.636)
(= (mission_duration Mission72) 175.769)
(= (mission_duration Mission73) 103.348)
(= (mission_duration Mission74) 115.766)
(= (mission_duration Mission75) 107.368)
(= (mission_duration Mission76) 93.333)
(= (mission_duration Mission77) 102.021)
(= (mission_duration Mission78) 114.632)
(= (mission_duration Mission79) 106.106)
(= (mission_duration Mission8) 128.408)
(= (mission_duration Mission80) 91.787)
(= (mission_duration Mission82) 160.024)
(= (mission_duration Mission83) 164.312)
(= (mission_duration Mission84) 180.286)
(= (mission_duration Mission85) 176.439)
(= (mission_duration Mission86) 159.274)
(= (mission_duration Mission87) 163.586)
(= (mission_duration Mission88) 179.636)
(= (mission_duration Mission89) 175.769)
(= (mission_duration Mission9) 154.759)
(= (mission_duration Mission90) 192.728)
(= (mission_duration Mission91) 176.092)
(= (mission_duration Mission92) 174.1)
(= (mission_duration Mission93) 190.98)
(= (mission_duration Mission94) 192.22)
(= (mission_duration Mission95) 175.514)
(= (mission_duration Mission96) 173.514)
(= (mission_duration Mission97) 190.466)
(= (mission_duration Mission98) 103.348)
(= (mission_duration Mission99) 115.766)

(connected mission_site_start_point_0 mission_site_start_point_1) (= (distance mission_site_start_point_0 mission_site_start_point_1) 53.8888)
(connected mission_site_start_point_0 wp_auv0) (= (distance mission_site_start_point_0 wp_auv0) 56.7891)
(connected wp_auv0 mission_site_start_point_0) (= (distance wp_auv0 mission_site_start_point_0) 56.7891)
(connected mission_site_start_point_1 mission_site_start_point_0) (= (distance mission_site_start_point_1 mission_site_start_point_0) 53.8888)
(connected mission_site_start_point_1 wp_auv0) (= (distance mission_site_start_point_1 wp_auv0) 22.561)
(connected wp_auv0 mission_site_start_point_1) (= (distance wp_auv0 mission_site_start_point_1) 22.561)

)
(:metric maximize (mission_total))
(:goal (> (mission_total) 0))
)
