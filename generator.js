function registerGenerator (Blockly) {

    Blockly.Arduino['badecar_init'] = function () {
        return '// 德中智慧小車初始化\n';
    };

    return Blockly;
}

exports = registerGenerator;
