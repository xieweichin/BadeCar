/* BadeCar generators v02 */
function registerGenerator(Blockly) {

    // 初始化
    Blockly.Arduino['badecar_init'] = function () {
        Blockly.Arduino.includes_['badecar_core'] = '#include <BadeCarCore.h>';
        Blockly.Arduino.definitions_['badecar_core_obj'] = 'BadeCarCore core;';
        return 'core.init();\n';
    };

    // 單一馬達 PWM
    Blockly.Arduino['badecar_motor_pwm'] = function (block) {
        var motor = block.getFieldValue('MOTOR_ID');
        var speed = Blockly.Arduino.valueToCode(block, 'SPEED', Blockly.Arduino.ORDER_ATOMIC) || 0;

        return 'core.setMotorPwm(' + motor + ', ' + speed + ');\n';
    };

    return Blockly;
}

exports = registerGenerator;
