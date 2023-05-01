function circlePath(center_x, center_y, r, skew){
    var dis = [
        "M", center_x, center_y+r,
        "A", r*skew,r, "0","1","0",center_x, center_y-r,
        "A", r*skew,r, "0","0","0",center_x, center_y+r,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}

function bezelCirclePath(center_x, center_y, r){
    var dis = [
        "M", center_x-r, center_y,
        "C", center_x-r, center_y+r, center_x+r, center_y+r, center_x+r, center_y,// bottom half
        "C", center_x+r, center_y-r, center_x-r, center_y-r, center_x-r, center_y,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}

function doubleArcPath(center_x, center_y, eright_x, eright_y, close){
    // v: 1 is bottom half, with higher close = more closed. 0 is upper half with lower close more closed
    var dis = [
        "M", center_x-eright_x*1.1, center_y,
        "C", center_x-eright_x*1.1, center_y+eright_y, center_x+eright_x*1.1, center_y+eright_y, center_x+eright_x*1.1, center_y,
        "Q", center_x, center_y+eright_y*(1-close)*2, center_x-eright_x*1.1, center_y,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}

export {circlePath, bezelCirclePath, doubleArcPath}