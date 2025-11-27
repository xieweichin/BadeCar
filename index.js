const badecar = formatMessage => ({
    name: '德中智慧小車',
    extensionId: 'badecar',

    version: '0.1.0',

    supportDevice: ['arduinoEsp32'],
    author: '八德國中 謝偉欽',

    iconURL: `assets/badecar.png`,

    description: '德中智慧小車 初始化模組（最小可用版本）',

    featured: true,

    blocks: 'blocks.js',
    generator: 'generator.js',
    toolbox: 'toolbox.js',
    translations: 'translations.js',
    msg: 'msg.json',

    categoryId: 'badecar',

    official: false,
    tags: ['shield'],
    helpLink: ''
});

module.exports = badecar;
