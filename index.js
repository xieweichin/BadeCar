const badecar = formatMessage => ({

    name: '德中智慧小車',
    extensionId: 'badecar',
    version: '0.2.0',

    supportDevice: ['arduinoEsp32'],
    author: '八德國中 謝偉欽',

    iconURL: `assets/badecar.png`,

    description: '德中智慧小車 基本控制模組（v02）',
    featured: true,

    blocks: 'blocks.js',
    generator: 'generator.js',
    toolbox: 'toolbox.js',
    translations: 'translations.js',
    library: 'lib',

    official: false,
    tags: ['shield'],
    helpLink: ''
});

module.exports = badecar;
