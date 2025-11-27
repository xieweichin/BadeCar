/* BadeCar basic blocks v02 */
function registerBlocks(Blockly) {
    const colour = '#b784b3';

    // 初始化積木
    Blockly.Blocks['badecar_init'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('德中智慧小車 初始化');
            this.setColour(colour);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
        }
    };

    // 單一馬達 PWM
    Blockly.Blocks['badecar_motor_pwm'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldDropdown([
                    ['M0', 'M0'],
                    ['M1', 'M1'],
                    ['M2', 'M2'],
                    ['M3', 'M3']
                ]), 'MOTOR_ID')
                .appendField('馬達 速度');
            
            this.appendValueInput("SPEED")
                .setCheck("Number");

            this.setColour(colour);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setTooltip('設定指定馬達的 PWM 速度 (-255 ~ 255)');
        }
    };

    return Blockly;
}

exports = registerBlocks;
