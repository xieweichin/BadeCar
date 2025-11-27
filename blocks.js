function registerBlocks (Blockly) {

    const colour = '#b784b3';

    Blockly.Blocks['badecar_init'] = {
        init: function () {
            this.jsonInit({
                "message0": "德中智慧小車 初始化",
                "colour": colour,
                "previousStatement": true,
                "nextStatement": true
            });
        }
    };

    return Blockly;
}

exports = registerBlocks;
