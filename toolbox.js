module.exports = {
    blocks: [
        {
            xml: `
<category name="德中智慧小車" colour="#b784b3">

    <label text="初始化"></label>
    <block type="badecar_init"></block>

    <label text="基本馬達控制"></label>
    <block type="badecar_motor_pwm">
        <value name="SPEED">
            <shadow type="math_number">
                <field name="NUM">255</field>
            </shadow>
        </value>
    </block>

</category>
`
        }
    ]
};
