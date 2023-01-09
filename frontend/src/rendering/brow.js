import { animated, useSpring } from "react-spring";


let cx, cy, ox, oy, mx, my, ix, iy

function AdjustMUnits (position, browAU) 
{
    var scale = position.auScaler;
    var flip = position.flip;
    var au1_inner_brow_raiser = browAU.au1*scale;
    var au2_outer_brow_raiser = browAU.au2*scale;
    var au4_brow_lowerer = browAU.au4*scale;


    [cx, cy] = [position.x, position.y];
    [ix, iy, mx, my, ox, oy] = [5*flip,.8, 0,1.3, -15*flip,1.5];
    // console.log([cx, cy, ox, oy, mx, my, ix, iy])
    my += my + au1_inner_brow_raiser*.25
    iy += iy + au1_inner_brow_raiser*.5

    my += my + au2_outer_brow_raiser*.5
    mx += mx - au2_outer_brow_raiser*.5*flip
    oy += oy + au2_outer_brow_raiser

    mx += mx + au4_brow_lowerer*flip
    my += my - au4_brow_lowerer
    ix += ix + au4_brow_lowerer*flip
    iy += iy - au4_brow_lowerer*1.4

    return [cx, cy, ox, oy, mx, my, ix, iy]
}

function Linify(bunits)
{
    [cx, cy, ox, oy, mx, my, ix, iy] = bunits
    var dis = [
    "M", cx+ox, cy-oy,
    "Q", 
    cx+mx, cy-my,
    cx+ix, cy-iy,
    
    "L",cx+ix*1.1, cy-iy-2,
    // "M", cx+ox, cy-oy,
    "Q", 
    cx+mx, cy-my-4,
    cx+ox, cy-oy-1,
    "Z"
    ];


    const d = dis.join(' ');
    // console.log(d)
    return d
}


function Brow({position, browAU}) {
   
    const bunits = AdjustMUnits(position, browAU)
    const d = Linify(bunits)
    const animationProps = useSpring({
        brow: d}
        )
  
    return (
          <animated.path d={animationProps.brow} stroke="#000" fill="#000" fillOpacity=".7" strokeWidth="0.3" strokeLinecap="round"/>
    );
}

export default Brow;



