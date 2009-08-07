#!/usr/bin/wish -f
proc scaler {n x} {
    set x2 [expr 100-$x]
    puts stdout "go 3 $n $x"
}

proc altscaler {n1 n2 x} {
    set x2 $x
    puts stdout "go $n1 $n2 $x2"
}

proc snd {glot_vol asp_vol} {
    .scroll_volume set $glot_vol
    .scroll_asp_volume set $asp_vol
}

proc fric {vol position center width} {
    .scroll_fric_volume set $vol
    .scroll_fric_position set $position
    .scroll_fric_center set $center
    .scroll_fric_width set $width
}

proc tube {r0 r1 r2 r3 r4 r5 r6 r7 velum} {
    .constrict.scroll_0 set $r0
    .constrict.scroll_1 set $r1
    .constrict.scroll_2 set $r2
    .constrict.scroll_3 set $r3
    .constrict.scroll_4 set $r4
    .constrict.scroll_5 set $r5
    .constrict.scroll_6 set $r6
    .constrict.scroll_7 set $r7
    .scroll_velum set $velum
}


frame .constrict

frame .phone

scale .constrict.scroll_0 \
    -command {scaler 0} -from 2 -to 0 -resolution 0.1

scale .constrict.scroll_1 \
    -command {scaler 1} -from 2 -to 0 -resolution 0.01

scale .constrict.scroll_2 \
    -command {scaler 2} -from 2 -to 0 -resolution 0.01

scale .constrict.scroll_3 \
    -command {scaler 3} -from 2 -to 0 -resolution 0.01

scale .constrict.scroll_4 \
    -command {scaler 4} -from 2 -to 0 -resolution 0.01

scale .constrict.scroll_5 \
    -command {scaler 5} -from 2 -to 0 -resolution 0.01

scale .constrict.scroll_6 \
    -command {scaler 6} -from 2 -to 0 -resolution 0.01

scale .constrict.scroll_7 \
    -command {scaler 7} -from 2 -to 0 -resolution 0.01

scale .scroll_volume \
    -command {altscaler 0 0} -orient horizontal -label "glottal"

scale .scroll_pitch \
    -command {altscaler 0 1} -orient horizontal

scale .scroll_asp_volume \
    -command {altscaler 1 0} -orient horizontal -label "aspiration"

scale .scroll_fric_volume \
    -command {altscaler 2 0} -orient horizontal -label "frication" \
    -from 0 -to 1 -resolution 0.01

scale .scroll_fric_position \
    -command {altscaler 2 1} -orient horizontal

scale .scroll_fric_center \
    -command {altscaler 2 2} -orient horizontal

scale .scroll_fric_width \
    -command {altscaler 2 3} -orient horizontal

scale .scroll_velum \
    -command {altscaler 4 0} -orient horizontal -label "velum" -from 0 -to 1 -resolution 0.05

scale .scroll_breathe \
    -command {altscaler 5 0} -orient horizontal -label "breathe"

scale .scroll_push \
    -command {altscaler 6 0} -orient horizontal -label "breath width"

scale .scroll_source \
    -command {altscaler 7 0} -orient horizontal -label "sound source"
button .push_qz -text "qz" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.9 0.2 0.6 1.31 0.1
	fric 0.0 5.8 55 5

snd 0.0 0.0
}

button .push_rr -text "rr" -width 5 -command {
	tube 0.8 1.31 0.73 1.31 2.12 0.63 1.78 0.65 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_y -text "y" -width 5 -command {
	tube 0.8 1.67 1.91 1.99 0.63 0.29 0.58 1.49 0.25
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_g -text "g" -width 5 -command {
	tube 0.8 1.7 1.3 0.99 0.1 1.07 0.73 1.49 0.1
	fric 0.0 4.7 20 20

snd 43.5 0.0
}

button .push_e -text "e" -width 5 -command {
	tube 0.8 0.68 1.12 1.695 1.385 1.07 1.045 2.06 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_ll -text "ll" -width 5 -command {
	tube 0.8 0.63 0.47 0.65 1.54 0.45 0.26 1.05 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_w -text "w" -width 5 -command {
	tube 0.8 1.91 1.44 0.6 1.02 1.33 1.56 0.55 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_ls -text "ls" -width 5 -command {
	tube 0.8 0.63 0.47 0.65 1.54 0.45 0.26 1.05 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_dh -text "dh" -width 5 -command {
	tube 0.8 1.2 1.5 1.35 1.2 1.2 0.4 1.0 0.1
	fric 0.25 6.0 44 45

snd 54.0 0.0
}

button .push_qp -text "qp" -width 5 -command {
	tube 0.8 0.89 0.76 1.28 1.8 0.99 0.84 0.1 0.1
	fric 0.0 7.0 20 7

snd 0.0 0.0
}

button .push_tx -text "tx" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.76 0.1 1.44 1.31 0.1
	fric 0.0 6.7 45 20

snd 0.0 0.0
}

button .push_ng -text "ng" -width 5 -command {
	tube 0.8 1.7 1.3 0.99 0.1 1.07 0.73 1.49 0.5
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_a -text "a" -width 5 -command {
	tube 0.8 0.65 0.65 0.65 1.31 1.23 1.31 1.67 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_ph -text "ph" -width 5 -command {
	tube 0.8 0.89 0.99 0.81 0.6 0.52 0.71 0.24 0.1
	fric 24.0 7.0 8.64 35.87

snd 0.0 0.0
}

button .push_dx -text "dx" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.76 0.1 1.44 1.31 0.1
	fric 0.0 6.7 45 20

snd 43.5 0.0
}

button .push_kx -text "kx" -width 5 -command {
	tube 0.8 1.7 1.3 0.99 0.1 1.07 0.73 1.49 0.1
	fric 0.0 4.7 20 20

snd 0.0 0.0
}

button .push_u -text "u" -width 5 -command {
	tube 0.8 0.625 0.6 0.705 1.12 1.93 1.515 0.625 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_k -text "k" -width 5 -command {
	tube 0.8 1.7 1.3 0.99 0.1 1.07 0.73 1.49 0.1
	fric 0.0 4.7 20 20

snd 0.0 0.0
}

button .push_in -text "in" -width 5 -command {
	tube 0.8 0.65 0.835 1.15 1.305 1.59 1.59 2.61 1.5
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_s -text "s" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.9 0.2 0.4 1.31 0.1
	fric 0.8 5.8 55 5

snd 0.0 0.0
}

button .push_qk -text "qk" -width 5 -command {
	tube 0.8 1.7 1.3 0.99 0.1 1.07 0.73 1.49 0.1
	fric 0.0 4.7 20 20

snd 0.0 0.0
}

button .push_uu -text "uu" -width 5 -command {
	tube 0.8 1.91 1.44 0.6 1.02 1.33 1.56 0.55 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_gs -text "gs" -width 5 -command {
	tube 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.1
	fric 0.0 5.5 25 5

snd 0.0 0.0
}

button .push_hv -text "hv" -width 5 -command {
	tube 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.1
	fric 0.0 5.5 25 5

snd 42.0 10.0
}

button .push_ee -text "ee" -width 5 -command {
	tube 0.8 1.67 1.905 1.985 0.81 0.495 0.73 1.485 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_i -text "i" -width 5 -command {
	tube 0.8 1.045 1.565 1.75 0.94 0.68 0.785 1.12 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_qt -text "qt" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.76 0.1 1.44 1.31 0.1
	fric 0.0 7.0 45 20

snd 0.0 0.0
}

button .push_m -text "m" -width 5 -command {
	tube 0.8 0.89 0.76 1.28 1.8 0.99 0.84 0.1 0.5
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_uh -text "uh" -width 5 -command {
	tube 0.8 0.89 0.99 0.81 0.76 1.05 1.23 1.12 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_o -text "o" -width 5 -command {
	tube 0.8 1.0 0.925 0.6 1.27 1.83 1.97 1.12 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_ah -text "ah" -width 5 -command {
	tube 0.8 0.65 0.45 0.94 1.1 1.52 1.46 2.45 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_d -text "d" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.76 0.1 1.44 1.3 0.1
	fric 0.0 6.7 45 20

snd 43.5 0.0
}

button .push_on -text "on" -width 5 -command {
	tube 0.8 1.0 0.925 0.6 1.265 1.83 1.965 1.12 1.5
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_aa -text "aa" -width 5 -command {
	tube 0.8 0.65 0.84 1.15 1.31 1.59 1.59 2.61 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_ov -text "ov" -width 5 -command {
	tube 0.8 0.885 0.99 0.81 0.755 1.045 1.225 1.12 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_b -text "b" -width 5 -command {
	tube 0.8 0.89 0.76 1.28 1.8 0.99 0.84 0.1 0.1
	fric 0.0 7.0 20 7

snd 43.5 0.0
}

button .push_q -text "q" -width 5 -command {
	tube 0.8 0.89 0.99 0.81 0.76 1.05 1.23 0.01 0.1
	fric 0.0 5.5 25 5

snd 0.0 0.0
}

button .push_z -text "z" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.9 0.2 0.6 1.31 0.1
	fric 0.8 5.8 55 5

snd 54.0 0.0
}

button .push_oh -text "oh" -width 5 -command {
	tube 0.8 0.885 0.99 0.81 0.755 1.045 1.225 1.12 0.1
	fric 0.0 5.5 25 5

snd 0.0 0.0
}

button .push_ch -text "ch" -width 5 -command {
	tube 0.8 1.36 1.74 1.87 0.94 0.0 0.79 0.79 0.1
	fric 0.0 5.6 25 26

snd 0.0 0.0
}

button .push_aw -text "aw" -width 5 -command {
	tube 0.8 1.1 0.94 0.42 1.49 1.67 1.78 1.05 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_qs -text "qs" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.9 0.2 0.4 1.31 0.1
	fric 0.0 5.8 55 5

snd 0.0 0.0
}

button .push_an -text "an" -width 5 -command {
	tube 0.8 0.52 0.45 0.79 1.49 1.67 1.02 1.59 1.5
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_qc -text "qc" -width 5 -command {
	tube 0.8 1.36 1.74 1.87 0.94 0.1 0.79 0.79 0.1
	fric 0.0 5.6 25 26

snd 0.0 0.0
}

button .push_bx -text "bx" -width 5 -command {
	tube 0.8 0.89 0.76 1.28 1.8 0.99 0.84 0.1 0.1
	fric 0.0 7.0 20 7

snd 43.5 0.0
}

button .push_j -text "j" -width 5 -command {
	tube 0.8 1.36 1.74 1.87 0.94 0.0 0.79 0.79 0.1
	fric 0.0 5.6 25 26

snd 48.0 0.0
}

button .push_sh -text "sh" -width 5 -command {
	tube 0.8 1.36 1.74 1.87 0.94 0.37 0.79 0.79 0.1
	fric 0.4 5.6 25 26

snd 0.0 0.0
}

button .push_t -text "t" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 0.76 0.1 1.44 1.31 0.1
	fric 0.0 7.0 45 20

snd 0.0 0.0
}

button .push_th -text "th" -width 5 -command {
	tube 0.8 1.2 1.5 1.35 1.2 1.2 0.4 1.0 0.1
	fric 0.25 6.0 44 45

snd 0.0 0.0
}

button .push_er -text "er" -width 5 -command {
	tube 0.8 0.885 0.99 0.81 0.755 1.045 1.225 1.12 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_v -text "v" -width 5 -command {
	tube 0.8 0.89 0.99 0.81 0.76 0.89 0.84 0.5 0.1
	fric 0.2 7.0 33 10

snd 54.0 0.0
}

button .push_px -text "px" -width 5 -command {
	tube 0.8 0.89 0.76 1.28 1.8 0.99 0.84 0.1 0.1
	fric 0.0 7.0 20 7

snd 0.0 0.0
}

button .push_un -text "un" -width 5 -command {
	tube 0.8 0.885 0.99 0.81 0.755 1.045 1.225 1.12 1.5
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_r -text "r" -width 5 -command {
	tube 0.8 1.31 0.73 1.07 2.12 0.47 1.78 0.65 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_x -text "x" -width 5 -command {
	tube 0.8 1.7 1.3 0.4 0.99 1.07 0.73 1.49 0.1
	fric 0.5 2.0 17.7 9

snd 0.0 0.0
}

button .push_h -text "h" -width 5 -command {
	tube 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.1
	fric 0.0 5.5 25 5

snd 0.0 10.0
}

button .push_f -text "f" -width 5 -command {
	tube 0.8 0.89 0.99 0.81 0.76 0.89 0.84 0.5 0.1
	fric 0.5 7.0 33 10

snd 0.0 0.0
}

button .push_n -text "n" -width 5 -command {
	tube 0.8 1.31 1.49 1.25 1.0 0.05 1.44 1.31 0.5
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_zh -text "zh" -width 5 -command {
	tube 0.8 1.36 1.74 1.87 0.94 0.37 0.79 0.79 0.1
	fric 0.4 5.6 25 26

snd 54.0 0.0
}

button .push_ar -text "ar" -width 5 -command {
	tube 0.8 0.52 0.45 0.79 1.49 1.67 1.02 1.59 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_l -text "l" -width 5 -command {
	tube 0.8 0.89 1.1 0.97 0.89 0.34 0.29 1.12 0.1
	fric 0.0 5.5 25 5

snd 60.0 0.0
}

button .push_p -text "p" -width 5 -command {
	tube 0.8 0.89 0.76 1.28 1.8 0.99 0.84 0.1 0.1
	fric 0.0 7.0 20 7

snd 0.0 0.0
}

button .push_hh -text "hh" -width 5 -command {
	tube 0.8 0.24 0.4 0.81 0.76 1.05 1.23 1.12 0.1
	fric 0.0 1.0 10 10

snd 0.0 10.0
}

grid .push_a -in .phone -row 1 -column 1
grid .push_aa -in .phone -row 1 -column 2
grid .push_ah -in .phone -row 1 -column 3
grid .push_an -in .phone -row 2 -column 1
grid .push_ar -in .phone -row 2 -column 2
grid .push_aw -in .phone -row 2 -column 3
grid .push_b -in .phone -row 3 -column 1
grid .push_bx -in .phone -row 3 -column 2
grid .push_ch -in .phone -row 3 -column 3
grid .push_d -in .phone -row 4 -column 1
grid .push_dh -in .phone -row 4 -column 2
grid .push_dx -in .phone -row 4 -column 3
grid .push_e -in .phone -row 5 -column 1
grid .push_ee -in .phone -row 5 -column 2
grid .push_er -in .phone -row 5 -column 3
grid .push_f -in .phone -row 6 -column 1
grid .push_g -in .phone -row 6 -column 2
grid .push_gs -in .phone -row 6 -column 3
grid .push_h -in .phone -row 7 -column 1
grid .push_hh -in .phone -row 7 -column 2
grid .push_hv -in .phone -row 7 -column 3
grid .push_i -in .phone -row 8 -column 1
grid .push_in -in .phone -row 8 -column 2
grid .push_j -in .phone -row 8 -column 3
grid .push_k -in .phone -row 9 -column 1
grid .push_kx -in .phone -row 9 -column 2
grid .push_l -in .phone -row 9 -column 3
grid .push_ll -in .phone -row 10 -column 1
grid .push_ls -in .phone -row 10 -column 2
grid .push_m -in .phone -row 10 -column 3
grid .push_n -in .phone -row 11 -column 1
grid .push_ng -in .phone -row 11 -column 2
grid .push_o -in .phone -row 11 -column 3
grid .push_oh -in .phone -row 12 -column 1
grid .push_on -in .phone -row 12 -column 2
grid .push_ov -in .phone -row 12 -column 3
grid .push_p -in .phone -row 13 -column 1
grid .push_ph -in .phone -row 13 -column 2
grid .push_px -in .phone -row 13 -column 3
grid .push_q -in .phone -row 14 -column 1
grid .push_qc -in .phone -row 14 -column 2
grid .push_qk -in .phone -row 14 -column 3
grid .push_qp -in .phone -row 15 -column 1
grid .push_qs -in .phone -row 15 -column 2
grid .push_qt -in .phone -row 15 -column 3
grid .push_qz -in .phone -row 16 -column 1
grid .push_r -in .phone -row 16 -column 2
grid .push_rr -in .phone -row 16 -column 3
grid .push_s -in .phone -row 17 -column 1
grid .push_sh -in .phone -row 17 -column 2
grid .push_t -in .phone -row 17 -column 3
grid .push_th -in .phone -row 18 -column 1
grid .push_tx -in .phone -row 18 -column 2
grid .push_u -in .phone -row 18 -column 3
grid .push_uh -in .phone -row 19 -column 1
grid .push_un -in .phone -row 19 -column 2
grid .push_uu -in .phone -row 19 -column 3
grid .push_v -in .phone -row 20 -column 1
grid .push_w -in .phone -row 20 -column 2
grid .push_x -in .phone -row 20 -column 3
grid .push_y -in .phone -row 21 -column 1
grid .push_z -in .phone -row 21 -column 2
grid .push_zh -in .phone -row 21 -column 3
pack .phone -side right -fill x


pack .constrict.scroll_7 -side right -fill y
pack .constrict.scroll_6 -side right -fill y
pack .constrict.scroll_5 -side right -fill y
pack .constrict.scroll_4 -side right -fill y
pack .constrict.scroll_3 -side right -fill y
pack .constrict.scroll_2 -side right -fill y
pack .constrict.scroll_1 -side right -fill y
pack .constrict.scroll_0 -side right -fill y
pack .constrict -side top -fill x

.constrict.scroll_7 set 1
.constrict.scroll_6 set 1
.constrict.scroll_5 set 1
.constrict.scroll_4 set 1
.constrict.scroll_3 set 1
.constrict.scroll_2 set 1
.constrict.scroll_1 set 1
.constrict.scroll_0 set 1

pack .scroll_source -side bottom -fill x
pack .scroll_push -side bottom -fill x
pack .scroll_breathe -side bottom -fill x
pack .scroll_velum -side bottom -fill x
pack .scroll_fric_width -side bottom -fill x
pack .scroll_fric_center -side bottom -fill x
pack .scroll_fric_position -side bottom -fill x
pack .scroll_fric_volume -side bottom -fill x
pack .scroll_asp_volume -side bottom -fill x
pack .scroll_pitch -side bottom -fill x
pack .scroll_volume -side bottom -fill x

