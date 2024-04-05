apply() {
    cp Auto.java Auto$1.java
    for p in ${@:2}; do
        patch Auto$1.java $p.diff
    done
    sed -i 's/Auto\([" ]\)/Auto'"$1"'\1/g' Auto$1.java
}

apply RedShort

apply RedShortPark nocycle2

apply RedFar long

apply RedFarPark long nocycle2

apply BlueShort blue

apply BlueShortPark blue nocycle2

apply BlueFar blue long

apply BlueFarPark blue long nocycle2
