/*serp_car V1.5 1024/7/8
超音波センサ
    I2C ADD:0x57
左・フォトリフレクター（B input)
    P1
右・フォトリフレクター(A input)
    P0
360サーボモーター
    左 P16
    右 P15

左ホイール・フォトセンサー(D input)
    P13
右ホイール・フォトセンサー(C input)
    P2
カラーセンサー
    I2C
*/

let drive_mode = 0
let step_mode = 0
let L_U = 0
let L_e_pre = 0
let L_ie = 0
let L_e = 0
let L_r = 0
let L_y = 0
let R_U = 0
let R_e_pre = 0
let R_ie = 0
let R_e = 0
let R_r = 0
let R_y = 0
let L_counter = 0
let R_counter = 0
let T = 0
let L_ki = 0
let L_kp = 0
let R_ki = 0
let R_kp = 0
let p = 0
let photo_sikii = 30
let u_dis = 0


R_kp = 30
R_ki = 20
L_kp = 30
L_ki = 20

// 制御周期分だけ待つ処理（ｍｓ）
T = 0.01
pins.setPull(DigitalPin.P13, PinPullMode.PullNone)
pins.setPull(DigitalPin.P2, PinPullMode.PullNone)

pins.setEvents(DigitalPin.P13, PinEventType.Pulse)
pins.setEvents(DigitalPin.P2, PinEventType.Pulse)

let noservo = 0

let color_value = 0
let volt = 0
let color_cycle = 10



pins.servoWritePin(AnalogPin.P15, 90)
pins.servoWritePin(AnalogPin.P16, 90)



//% color="#ff4500" weight=94 
namespace serp {

    export enum direction {
        //% block="forward"
        forward,
        //% block="right",
        right,
        //% block="left",
        left,
        //% block="right_rotation",
        right_rotation,
        //% block="left_rotation",
        left_rotation,
        //% block="backward",
        backward,
        //% block="Stop",
        Stop
    }

    export enum direction2 {
        //% block="forward"
        forward,
        //% block="right_rotation",
        right_rotation,
        //% block="left_rotation",
        left_rotation,
        //% block="backward",
        backward
    }

    export enum direction3 {
        //% block="forward"
        forward,
        //% block="backward",
        backward
    }

    export enum direction4 {
        //% block="right_rotation",
        right_rotation,
        //% block="left_rotation",
        left_rotation
    }


    let volt = pins.analogReadPin(AnalogPin.P1) / 500 * 6

    export enum kyori {
        //% block="long"
        long,
        //% block="short",
        short
    }




    export enum onoff {
        //% block="ON"
        ON,
        //% block="OFF"
        OFF
    }
    export enum whiteblack {
        //% block="black"
        black,
        //% block="white"
        white
    }



    export enum BT {
        //% block="lighter"
        lighter,
        //% block="darker"
        darker
    }




    export enum EventSide {
        //% block="left_onle"
        Left_onle,
        //% block="Right_onle"
        Right_onle,
        //% block="Both"
        Both
    }


    export enum colorcycle {
        //% block="cycle1"
        cycle1,
        //% block="cycle10",
        cycle10,
        //% block="cycle42",
        cycle42,
        //% block="cylce64",
        cycle64
    }

    export enum colorgain {
        //% block="1×gain"
        gain1,
        //% block="4×gain",
        gain4,
        //% block="16×gain",
        gain16,
        //% block="60×gain",
        gain60
    }

    export enum color_scene {
        //% block="Red"
        Red,
        //% block="Green",
        Green,
        //% block="Blue",
        Blue,
        //% block="Black",
        Black,
        //% block="White"
        White
    }


    export enum color_senser {
        //% block="Red"
        Red,
        //% block="Green",
        Green,
        //% block="Blue",
        Blue,
        //% block="Light",
        Light
    }

    export enum ServoPin {
        P3 = AnalogPin.P3,
        P4 = AnalogPin.P4,
        P8 = AnalogPin.P8,
        P14 = AnalogPin.P14
    }

    export enum LEDSide {
        //% block="left"
        Left,
        //% block="Right"
        Right,
        //% block="Both"
        Both
    }


    pins.onPulsed(DigitalPin.P2, PulseValue.High, function () {
        R_counter += 1
    })
    pins.onPulsed(DigitalPin.P2, PulseValue.Low, function () {
        R_counter += 1
    })
    pins.onPulsed(DigitalPin.P13, PulseValue.High, function () {
        L_counter += 1
    })
    pins.onPulsed(DigitalPin.P13, PulseValue.Low, function () {
        L_counter += 1
    })

    //% color="#1E90FF" weight=93 block="Wait time (sec)|%second|" group="1 Basic movement"
    //% second.min=0 second.max=10 second.defl=1
    export function driveForwards(second: number): void {
        basic.pause(second * 1000);
    }



    //% color="#3943c6" weight=90 
    //% block="Move |%sinkou_houkou|,power|%Power|" group="1 Basic movement"
    //% Power.min=0 Power.max=100 Power.defl=100
    export function car_derection(sinkou_houkou: direction, Power: number): void {
        //pins.setEvents(DigitalPin.P6, PinEventType.None)
        //pins.setEvents(DigitalPin.P7, PinEventType.None)
        switch (sinkou_houkou) {
            case direction.forward:
                cont_forward(Power)
                break;
            case direction.backward:
                cont_backward(Power)
                break;
            case direction.left:
                cont_L(Power)
                break;
            case direction.right:
                cont_R(Power)
                break;
            case direction.right_rotation:
                cont_Rrotate(Power)
                break;
            case direction.left_rotation:
                cont_Lrotate(Power)
                break;
            case direction.Stop:
                cont_stop(0)
                break;
        }
    }

    //% color="#3943c6" weight=88 
    //% block="Move |%sinkou_houkou|, |%time_sec|(sec) ,power|%Power|" group="1 Basic movement"
    //% Power.min=0 Power.max=100 Power.defl=100
    //% time_sec.min=0 time_sec.max=10 time_sec.defl=0
    export function car_derection2(sinkou_houkou: direction, time_sec: number, Power: number): void {
        //pins.setEvents(DigitalPin.P6, PinEventType.None)
        //pins.setEvents(DigitalPin.P7, PinEventType.None)
        switch (sinkou_houkou) {
            case direction.forward:
                cont_forward(Power)
                basic.pause(time_sec * 1000);
                cont_stop(Power)
                break;
            case direction.left:
                cont_L(Power)
                basic.pause(time_sec * 1000);
                cont_stop(Power)
                break;
            case direction.right:
                cont_R(Power)
                basic.pause(time_sec * 1000);
                cont_stop(Power)
                break;
            case direction.right_rotation:
                cont_Rrotate(Power)
                basic.pause(time_sec * 1000);
                cont_stop(Power)
                break;
            case direction.left_rotation:
                cont_Lrotate(Power)
                basic.pause(time_sec * 1000);
                cont_stop(Power)
                break;
            case direction.backward:
                cont_backward(Power)
                basic.pause(time_sec * 1000);
                cont_stop(Power)
                break;
            case direction.Stop:
                cont_stop(Power)
                basic.pause(time_sec * 1000);
                cont_stop(Power)
                break;
        }
    }



    //% color="#3943c6" weight=80 
    //% block="Move |%sinkou_houkou|,|%step|step" group="1 Basic movement"
    //% step.min=0 step.max=100 Power.defl=0
    export function car_stepmove(sinkou_houkou: direction2, step: number): void {
        switch (sinkou_houkou) {
            case direction2.forward:
                step_forward(step)
                break;
            case direction2.backward:
                step_backward(step)
                break;
            case direction2.left_rotation:
                step_Lrotate(step)
                break;
            case direction2.right_rotation:
                step_Rrotate(step)
                break;
        }
    }

    //% color="#3943c6" weight=85
    //% block="Move |%sinkou_houkou|,|%step|cm" group="1 Basic movement"
    //% step.min=0 step.max=50 Power.defl=0
    export function car_stepmove3(sinkou_houkou: direction3, step: number): void {
        switch (sinkou_houkou) {
            case direction3.forward:
                step_forward(step * 4.1)
                break;
            case direction3.backward:
                step_backward(step * 4.1)
                break;
        }
    }


    //% color="#3943c6" weight=83
    //% block="Move |%sinkou_houkou|,|%step|degree" group="1 Basic movement"
    //% step.min=0 step.max=180 Power.defl=90
    export function car_stepmove4(sinkou_houkou: direction4, step: number): void {
        switch (sinkou_houkou) {
            case direction4.right_rotation:
                step_Rrotate(step * 0.355)
                break;
            case direction4.left_rotation:
                step_Lrotate(step * 0.355)
                break;
        }
    }



    function step_forward(step_number: number) {
        cont_stop(0)
        basic.pause(50);
        step_mode = 1
        L_counter = 0
        R_counter = 0
        drive_mode = 0                                //左右の回転数ずれ　修正モードをON
        step_action(step_number)
    }
    function step_backward(step_number: number) {
        cont_stop(0)
        basic.pause(50);
        step_mode = 2
        L_counter = 0
        R_counter = 0
        drive_mode = 0
        step_action(step_number)
    }
    function step_Lrotate(step_number: number) {
        cont_stop(0)
        basic.pause(50);
        step_mode = 3
        L_counter = 0
        R_counter = 0
        drive_mode = 0
        step_action(step_number)
    }
    function step_Rrotate(step_number: number) {
        cont_stop(0)
        basic.pause(50);
        step_mode = 4
        L_counter = 0
        R_counter = 0
        drive_mode = 0
        step_action(step_number)
    }



    function step_action(STEP: number) {

        R_counter = 0
        L_counter = 0
        R_e_pre = 0
        L_e_pre = 0
        R_ie = 0
        L_ie = 0

        while (true) {
            basic.pause(10)
            // 現時刻の情報取得
            R_y = R_counter
            // 目標値
            R_r = STEP
            // PID制御の式
            R_e = R_r - R_y
            R_ie = R_ie + (R_e + R_e_pre) * T / 2
            R_U = R_kp * R_e + R_ki * R_ie
            if (R_U >= 600) {
                R_U = 600
            } else if (R_U <= 0) {
                R_U = 0
            }
            // 現時刻の情報取得
            L_y = L_counter
            // 目標値
            L_r = STEP
            // PID制御の式
            L_e = L_r - L_y
            L_ie = L_ie + (L_e + L_e_pre) * T / 2
            L_U = L_kp * L_e + L_ki * L_ie
            if (L_U >= 600) {
                L_U = 600
            } else if (L_U <= 0) {
                L_U = 0
            }
            //if (L_counter<15 || R_counter<15 ){
            //R_U=R_U/3
            //L_U=L_U/3
            //}
            p = (R_U + L_U) / 2
            //serial.writeNumbers([L_counter, L_U])
            if (L_counter > R_counter) {
                L_U = L_U * 0.6
                R_U = R_U * 1
            }
            else {
                L_U = L_U * 1
                R_U = R_U * 0.6
            }
            //serial.writeNumbers([L_counter, R_counter,R_U,L_U])
            if (step_mode == 1) {                         //前

                pins.servoWritePin(AnalogPin.P16, 90 + 90 * L_U / 1000)
                pins.servoWritePin(AnalogPin.P15, 90 - 90 * R_U / 1000)

            } else {
                if (step_mode == 2) {                      //後ろ
                    pins.servoWritePin(AnalogPin.P16, 90 - 90 * L_U / 1000)
                    pins.servoWritePin(AnalogPin.P15, 90 + 90 * R_U / 1000)
                } else {
                    if (step_mode == 3) {                  //左回転
                        pins.servoWritePin(AnalogPin.P16, 90 - 90 * L_U / 1000)
                        pins.servoWritePin(AnalogPin.P15, 90 - 90 * R_U / 1000)
                    } else {
                        if (step_mode == 4) {                  //右回転
                            pins.servoWritePin(AnalogPin.P16, 90 + 90 * L_U / 1000)
                            pins.servoWritePin(AnalogPin.P15, 90 + 90 * R_U / 1000)
                        }
                    }
                }
            }
            R_e_pre = R_e
            L_e_pre = L_e
            if (STEP <= L_y && STEP <= R_y) {
                pins.servoWritePin(AnalogPin.P16, 90)
                pins.servoWritePin(AnalogPin.P15, 90)
                drive_mode = 0
                basic.pause(200)
                break;
            }
        }
    }



    function cont_forward(power: number) {
        if (drive_mode != 1) {
            L_counter = 0
            R_counter = 0
            drive_mode = 1
        }                                //左右の回転数ずれ　修正モードをON
        p = power
        //pins.servoWritePin(AnalogPin.P16, 90+90*p/100)
        //pins.servoWritePin(AnalogPin.P15, 90-90*p/100)
    }
    function cont_backward(power: number) {
        if (drive_mode != 2) {
            L_counter = 0
            R_counter = 0
            drive_mode = 2
        }                              //左右の回転数ずれ　修正モードをON
        p = power
        //pins.servoWritePin(AnalogPin.P16, 90-90*p/100)
        //pins.servoWritePin(AnalogPin.P15, 90+90*p/100)
    }

    function cont_Lrotate(power: number) {
        L_counter = 0
        R_counter = 0
        p = power
        drive_mode = 3
        //pins.servoWritePin(AnalogPin.P16, 90-90*p/100)
        //pins.servoWritePin(AnalogPin.P15, 90-90*p/100)
    }
    function cont_Rrotate(power: number) {
        L_counter = 0
        R_counter = 0
        drive_mode = 4
        p = power
        //pins.servoWritePin(AnalogPin.P16, 90+90*p/100)
        //pins.servoWritePin(AnalogPin.P15, 90+90*p/100)
    }
    function cont_L(power: number) {
        drive_mode = 5
        p = power
        pins.servoWritePin(AnalogPin.P16, 90)
        pins.servoWritePin(AnalogPin.P15, 90 - 90 * p / 100)
    }
    function cont_R(power: number) {
        drive_mode = 6
        p = power
        pins.servoWritePin(AnalogPin.P16, 90 + 90 * p / 100)
        pins.servoWritePin(AnalogPin.P15, 90)
    }

    function cont_stop(power: number) {
        if (drive_mode != 7)
            p = 0
        pins.servoWritePin(AnalogPin.P16, 90)
        pins.servoWritePin(AnalogPin.P15, 90)
        drive_mode = 7
    }

    /**
    * 障がい物までの距離を返します
    */
    //% color="#009A00" weight=22 blockId=sonar_ping_2 block="Distance sensor" group="2 Ultrasonic_Distance sensor"

    export function sonar_ping_2(): number {
        // I2C　1を書き込み、120ｍS後に取得する
        let SR04_I2C_ADDR = 0x57
        let distance = 0
        while (distance == 0) {

            pins.i2cWriteNumber(SR04_I2C_ADDR, 1, NumberFormat.UInt8BE)
            basic.pause(120)
            let readbuf = pins.i2cReadBuffer(SR04_I2C_ADDR, pins.sizeOf(NumberFormat.UInt8LE) * 3)


            distance = (readbuf[0] * 65536 + readbuf[1] * 256 + readbuf[2]) / 10000;
            //if (distance<3){
            //    distance = 3
            //}
        }
        return (Math.round(distance))

    }




    //% color="#009A00" weight=30 block="(minimam 5cm) dstance |%limit| cm  |%nagasa| " group="2 Ultrasonic_Distance sensor"
    //% limit.min=5 limit.max=30 limit.defl=5

    export function sonar_ping_3(limit: number, nagasa: kyori): boolean {
        let d1 = u_dis
        switch (nagasa) {
            case kyori.short:
                if (d1 < limit) {
                    return true;
                } else {
                    return false;
                }
                break;
            case kyori.long:
                if (d1 < limit) {
                    return false;
                } else {
                    return true;
                }
                break;
        }
    }

    function photosenser() {
        let L = Math.round((pins.analogReadPin(AnalogPin.P0) / 1023) * 100)
        let R = Math.round((pins.analogReadPin(AnalogPin.P1) / 1023) * 100)

        return [L, R]


    }

    // しきい値を設定する関数
    //% block="set photo sensor threshold to %threshold"
    //% threshold.min=10 threshold.max=90 threshold.defl=30
    //% group="3 photoreflector"
    export function setPhotoSikii(threshold: number) {
        photo_sikii = threshold;
    }



    //% color="#696969" weight=28 blockId=auto_photo_R block="right_photoreflector" group="3 photoreflector"
    export function phto_R() {
        let reflecter_value = photosenser()
        return reflecter_value[0];
    }

    //% color="#696969" weight=28 blockId=auto_photo_L block="left_photoreflector" group="3 photoreflector"

    export function phto_L() {
        let reflecter_value = photosenser()
        return reflecter_value[1];
    }

    //% color="#696969"  weight=33 block="Photo Sensor |%pos| |%wb| " group="3 photoreflector"
    //% sence.min=10 sence.max=40
    export function photoSensor(pos: EventSide, wb: whiteblack): boolean {
        switch (pos) {
            case EventSide.Right_onle:
                switch (wb) {
                    case whiteblack.black:
                        return (phto_R() < photo_sikii && phto_L() > photo_sikii);
                    case whiteblack.white:
                        return (phto_R() > photo_sikii && phto_L() < photo_sikii);
                }
                break;
            case EventSide.Left_onle:
                switch (wb) {
                    case whiteblack.black:
                        return (phto_L() < photo_sikii && phto_R() > photo_sikii);
                    case whiteblack.white:
                        return (phto_L() > photo_sikii && phto_R() < photo_sikii);
                }
                break;
            case EventSide.Both:
                switch (wb) {
                    case whiteblack.black:
                        return (phto_R() < photo_sikii && phto_L() < photo_sikii);
                    case whiteblack.white:
                        return (phto_R() > photo_sikii && phto_L() > photo_sikii);
                }
                break;
        }
        return false;
    }


    /**
     * ラインセンサーの左右と白黒を判断させるブロック
     */
    //% block="when |%side| is |%wb|, do"
    //% color="#696969"  weight=32  " group="3 photoreflector"
    //% side.shadow="dropdown" side.options="EventSide"
    //% wb.shadow="dropdown" wb.options="whiteblack"
    export function onEvent(side: EventSide, wb: whiteblack, handler: () => void) {
        control.inBackground(function () {
            while (true) {
                let leftCondition = (wb === whiteblack.black) ? (phto_L() < photo_sikii) : (phto_L() > photo_sikii);
                let rightCondition = (wb === whiteblack.black) ? (phto_R() < photo_sikii) : (phto_R() > photo_sikii);

                switch (side) {
                    case EventSide.Left_onle:
                        if (leftCondition && !rightCondition) {
                            handler();
                        }
                        break;
                    case EventSide.Right_onle:
                        if (rightCondition && !leftCondition) {
                            handler();
                        }
                        break;
                    case EventSide.Both:
                        if (leftCondition && rightCondition) {
                            handler();
                        }
                        break;
                }
                basic.pause(20);
            }
        });
    }



    /**
     * Set the angle of the servo motor connected to the selected pin.
     * @param pin The pin where the servo motor is connected.
     * @param angle The angle to set the servo motor to, eg: 90
     */
    //% blockId=servo_motor_set_angle group="4 servo moter"
    //% block="set servo motor on pin %pin to %angle degrees"
    //% angle.min=0 angle.max=180
    //% inlineInputMode=inline
    export function setServoAngle(pin: ServoPin, angle: number): void {
        pins.servoWritePin(<AnalogPin><number>pin, angle);
    }









    /*
        smbus.writeByte(0x81, 0x00)  //0x81=10000001  RGB timing 700ms
        smbus.writeByte(0x81, 0x10)  //16×gain
    
        smbus.writeByte(0x80, 0x03)  //0x03を書くと動作開始
        smbus.writeByte(0x81, 0x2b)  //this.addr 0x29 0x81=10000001 0x2b=00101011
    */
    smbus.writeByte(0x81, 0xF6)  //cycle10


    smbus.writeByte(0x80, 0x03)  //0x03を書くと動作開始










    //% color="#ffa500"  weight=35 block="values |%color|" group="5 color senser"
    export function colorsenser_value(color: color_senser): number {
        switch (color) {
            case color_senser.Light:
                return color_L()
            case color_senser.Red:
                return color_R()
            case color_senser.Green:
                return color_G()
            case color_senser.Blue:
                return color_B()
        }
    }


    function color_R() {
        return Math.round(rgb()[0] / Math.max(rgb()[0], Math.max(rgb()[1], rgb()[2])) * 256)
    }
    function color_G() {
        return Math.round(rgb()[1] / Math.max(rgb()[0], Math.max(rgb()[1], rgb()[2])) * 256)
    }
    function color_B() {
        return Math.round(rgb()[2] / Math.max(rgb()[0], Math.max(rgb()[1], rgb()[2])) * 256)
    }
    function color_L() {
        let result: Buffer = smbus.readBuffer(0xb4, pins.sizeOf(NumberFormat.UInt16LE) * 4)
        return smbus.unpack("HHHH", result)[0] / color_cycle * 10
    }









    export function rgb(): number[] {
        let result: number[] = raw()
        let clear: number = result.shift()
        for (let x: number = 0; x < result.length; x++) {
            result[x] = result[x] * 255 / clear
        }
        return result
    }


    export function raw(): number[] {

        let result: Buffer = smbus.readBuffer(0xb4, pins.sizeOf(NumberFormat.UInt16LE) * 4)
        return smbus.unpack("HHHH", result)
    }


    //% color="#ffa500"  weight=16 blockId=color_temp block="color Temperatures value" group="5 color senser"
    export function color_temp(): number {
        return Math.round(3810 * color_B() / color_R() + 1391)
    }


    //% color="#ffa500" weight=88 blockId=selectcycle
    //% block="choice |%cycle|" group="5 color senser"
    export function selectcycle(cycle: colorcycle): void {

        switch (cycle) {
            case colorcycle.cycle1:
                color_cycle = 1
                smbus.writeByte(0x81, 0xFF)
                break;
            case colorcycle.cycle10:
                color_cycle = 10
                smbus.writeByte(0x81, 0xF6)
                break;
            case colorcycle.cycle42:
                color_cycle = 42
                smbus.writeByte(0x81, 0xD5)
                break;
            case colorcycle.cycle64:
                color_cycle = 64
                smbus.writeByte(0x81, 0xC0)
                break;
        }
    }



    //% color="#ffa500"  weight=16 blockId=color_ID block="color ID" group="5 color senser"
    export function color_ID(): number {
        /*     黒:0　　赤：1　緑：2　青：3　白:4  */
        let color_ID = 0
        /*io_neo.neopixel.colors(NeoPixelColors.Black))*/
        let R = color_R()
        let G = color_G()
        let B = color_B()
        let L = color_L()
        let total = R + G + B
        let normalized_r = R / total
        let normalized_g = G / total
        let normalized_b = B / total
        let maxcolor_value = Math.max(R, Math.max(G, B))
        if (R > 255 && L > 80 && L < 200) {//&& normalized_r > 0.45 && normalized_g < 0.35 && normalized_b < 0.35 && L>80 && L<200){
            color_ID = 1
        }
        else if (G > 255 && L > 80 && L < 200) {//&&normalized_g > 0.37 && normalized_r < 0.35 && normalized_b < 0.35 && L>80 && L<200){
            color_ID = 2
        }
        else if (B > 255 && L > 80 && L < 200) {//&& normalized_b > 0.30 && normalized_r < 0.40 && normalized_g < 0.40 &&  L>80 && L<200){
            color_ID = 3
        }
        else if (L > 250) {
            color_ID = 4
        }
        else if (L < 50) {
            color_ID = 0
        }
        return (color_ID);
    }





    //% color="#ffa500"  weight=14 block="|%scene|color " group="5 color senser"
    export function color1(scene: color_scene): boolean {

        switch (scene) {
            case color_scene.Red:
                if (color_ID() == 1) {
                    return true;
                } else {
                    return false;
                }
                break;
            case color_scene.Green:
                if (color_ID() == 2) {
                    return true;
                } else {
                    return false;
                }
                break;
            case color_scene.Blue:
                if (color_ID() == 3) {
                    return true;
                } else {
                    return false;
                }
                break;
            case color_scene.Black:
                if (color_ID() == 0) {
                    return true;
                } else {
                    return false;
                }
                break;
            case color_scene.White:
                if (color_ID() == 4) {
                    return true;
                } else {
                    return false;
                }
                break;
        }
    }




    namespace smbus {
        export function writeByte(register: number, value: number): void {
            let temp = pins.createBuffer(2);
            temp[0] = register;
            temp[1] = value;
            pins.i2cWriteBuffer(0x29, temp, false);
        }


        export function readBuffer(register: number, len: number): Buffer {
            let temp = pins.createBuffer(1);
            temp[0] = register;
            pins.i2cWriteBuffer(0x29, temp, false);
            return pins.i2cReadBuffer(0x29, len, false);
        }


        export function unpack(fmt: string, buf: Buffer): number[] {
            let le: boolean = true;
            let offset: number = 0;
            let result: number[] = [];
            let num_format: NumberFormat = 0;
            for (let c = 0; c < fmt.length; c++) {
                switch (fmt.charAt(c)) {
                    case '<':
                        le = true;
                        continue;
                    case '>':
                        le = false;
                        continue;
                    case 'c':
                    case 'B':
                        num_format = le ? NumberFormat.UInt8LE : NumberFormat.UInt8BE; break;
                    case 'b':
                        num_format = le ? NumberFormat.Int8LE : NumberFormat.Int8BE; break;
                    case 'H':
                        num_format = le ? NumberFormat.UInt16LE : NumberFormat.UInt16BE; break;
                    case 'h':
                        num_format = le ? NumberFormat.Int16LE : NumberFormat.Int16BE; break;
                }
                result.push(buf.getNumber(num_format, offset));
                offset += pins.sizeOf(num_format);
            }
            return result;
        }
    }






    //% color="#a0522d" weight=36 block="Human in motion" group="5 i:o Human detecting sensor"
    //% advanced=true
    export function IO_humanDetection(): boolean {
        pins.setPull(DigitalPin.P14, PinPullMode.PullNone);
        if (pins.digitalReadPin(DigitalPin.P14) == 1) {
            return true;
        } else {
            return false;
        }
    }

    //% color="#a0522d" weight=34 blockId=IO_human block="human detecting sensor binary" group="5 i:o Human detecting sensor"
    //% advanced=true
    export function IO_human(): number {
        pins.setPull(DigitalPin.P14, PinPullMode.PullNone);
        return pins.digitalReadPin(DigitalPin.P14);
    }

    //% color="#a0522d"  weight=79 blockId=IO_human_DISP block="Show human detecting sensor binary" group="5 i:o Human detecting sensor"
    //% advanced=true
    export function IO_human_DISP() {
        pins.setPull(DigitalPin.P14, PinPullMode.PullNone);
        basic.showNumber(pins.digitalReadPin(DigitalPin.P14));
    }

    //% color="#009A00"  weight=81 blockId=microbit2_decideLight block="Light sensor value (than|%limit|) |%brightness| " group="6 micro:bit light sensor"
    //% limit.min=0 limit.max=100
    //% advanced=true
    export function microbit2_decideLight(limit: number, Brightness: BT): boolean {
        switch (Brightness) {
            case BT.darker:
                if (input.lightLevel() / 254 * 100 <= limit) {
                    return true;
                } else {
                    return false;
                }
            case BT.lighter:
                if (input.lightLevel() / 254 * 100 < limit) {
                    return false;
                } else {
                    return true;
                }
        }
    }



    //% color="#009A00"  weight=80 blockId=microbit2_denkitemp block="Light sensor value" group="6 micro:bit light sensor"
    //% advanced=true
    export function microbit2_denkitemp(): number {
        return Math.round(input.lightLevel() / 254 * 100);
    }


    //% color="#228b22"  weight=82 blockId=microbit2_denkiLED block="Show light sensor values" group="6 micro:bit light sensor"
    //% advanced=true
    export function microbit2_denkiLED() {
        basic.showNumber(Math.round(input.lightLevel() / 254 * 100));
    }


    //% color="#4682b4" weight=58 blockId=IO_relay block="Relay (digital) write|%mode|" group="7 i:o Relay control"
    //% advanced=true
    export function IO_relay(mode: onoff) {
        switch (mode) {
            case onoff.ON: {
                return pins.digitalWritePin(DigitalPin.P8, 1);
            }
            case onoff.OFF: {
                return pins.digitalWritePin(DigitalPin.P8, 0);
            }
        }
    }

    //% color="#4682b4" weight=56 blockId=IO_relay_2 block="Relay (analog) write|%syuturyoku|" group="7 i:o Relay control"
    //% syuturyoku.min=0 syuturyoku.max=1023 syuturyoku.defl=1023
    //% advanced=true
    export function IO_relay_2(syuturyoku: number) {
        return pins.analogWritePin(AnalogPin.P8, syuturyoku);
    }



    loops.everyInterval(120, function () {
        u_dis = serp.sonar_ping_2()
    })


    basic.forever(function () {
        //serial.writeNumbers([L_counter, R_counter])
        let n1 = 0
        let n2 = 0
        if (p < 70) {
            n1 = 1
            n2 = 1.4
        }
        else {
            n1 = 0.6
            n2 = 1
        }
        //前進モード
        if (drive_mode == 1) {
            if (L_counter < R_counter) {
                pins.servoWritePin(AnalogPin.P16, 90 + 90 * p * n2 / 100)
                pins.servoWritePin(AnalogPin.P15, 90 - 90 * p * n1 / 100)
                //serial.writeLine("L<R")
                //serial.writeNumbers([L_counter ,R_counter])
                //                pins.analogWritePin(AnalogPin.P2, p*n2)
                //                pins.analogWritePin(AnalogPin.P16, p*n1)
            } else {
                pins.servoWritePin(AnalogPin.P16, 90 + 90 * p * n1 / 100)
                pins.servoWritePin(AnalogPin.P15, 90 - 90 * p * n2 / 100)
                //                pins.analogWritePin(AnalogPin.P2, p*n1)
                //                pins.analogWritePin(AnalogPin.P16, p)
                //serial.writeLine("L>R")
                //serial.writeNumbers([L_counter ,R_counter])
            }
        }
        //後進モード
        if (drive_mode == 2) {
            if (L_counter < R_counter) {
                pins.servoWritePin(AnalogPin.P16, 90 - 90 * p * n2 / 100)
                pins.servoWritePin(AnalogPin.P15, 90 + 90 * p * n1 / 100)
                //                pins.analogWritePin(AnalogPin.P13, p*n2)
                //                pins.analogWritePin(AnalogPin.P15, p*n1)
            } else {
                pins.servoWritePin(AnalogPin.P16, 90 - 90 * p * n1 / 100)
                pins.servoWritePin(AnalogPin.P15, 90 + 90 * p * n2 / 100)
                //                pins.analogWritePin(AnalogPin.P13, p*n1)
                //                pins.analogWritePin(AnalogPin.P15, p*n2)
            }
        }
        //左回転モード
        if (drive_mode == 3) {
            if (L_counter < R_counter) {
                pins.servoWritePin(AnalogPin.P16, 90 - 90 * p * n2 / 100)
                pins.servoWritePin(AnalogPin.P15, 90 - 90 * p * n1 / 100)
                //                pins.analogWritePin(AnalogPin.P13, p*n2)
                //                pins.analogWritePin(AnalogPin.P16, p*n1)
            } else {
                pins.servoWritePin(AnalogPin.P16, 90 - 90 * p * n1 / 100)
                pins.servoWritePin(AnalogPin.P15, 90 - 90 * p * n2 / 100)
                //                pins.analogWritePin(AnalogPin.P13, p*n1)
                //                pins.analogWritePin(AnalogPin.P16, p*n2)
            }
        }
        //右回転モード
        if (drive_mode == 4) {
            if (L_counter < R_counter) {
                pins.servoWritePin(AnalogPin.P16, 90 + 90 * p * n2 / 100)
                pins.servoWritePin(AnalogPin.P15, 90 + 90 * p * n1 / 100)
                //                pins.analogWritePin(AnalogPin.P2, p*n2)
                //                pins.analogWritePin(AnalogPin.P15, p*n1)
            } else {
                pins.servoWritePin(AnalogPin.P16, 90 + 90 * p * n1 / 100)
                pins.servoWritePin(AnalogPin.P15, 90 + 90 * p * n2 / 100)
                //                pins.analogWritePin(AnalogPin.P2, p*n1)
                //                pins.analogWritePin(AnalogPin.P15, p*n2)
            }
        }
        //serial.writeNumbers([drive_mode,p,90+90*p*n2/100,90-90*p*n1/100,90+90*p*n1/100,90-90*p*n2/100,L_counter, R_counter])
    })

}
