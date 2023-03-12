import React from "react"

class EyeForm extends React.Component {
    constructor(props) {
        super(props);
        this.v = props.v
  
    this.handleChange = this.handleChange.bind(this);
    this.handleSubmit = this.handleSubmit.bind(this);
}
  
    handleChange(event) {
        let temp = {}
        temp[event.target.id] = event.target.value
        this.props.f(temp);
        this.v[event.target.id] = event.target.value
    }
  
    handleSubmit(event) {
      event.preventDefault();
    }
  
    render() {
      return (
        <form onSubmit={this.handleSubmit}>
          <label>Eyes:</label>
            <br></br>
            <label>au5_ Upper Lid Raiser:</label>
            <input type="range" 
                id='au5_upper_lid_raiser'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au5_upper_lid_raiser} 
                onChange={this.handleChange} />
            {this.v.au5_upper_lid_raiser}
            <br></br>
            <label>au6_ Cheek Raiser:</label>
            <input type="range" 
                id='au6_cheek_raiser'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au6_cheek_raiser} 
                onChange={this.handleChange} />
            {this.v.au6_cheek_raiser}
            <br></br>
            <label>au7_ Lid Tightener:</label>
            <input type="range" 
                id='au7_lid_tightener'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au7_lid_tightener} 
                onChange={this.handleChange} />
            {this.v.au7_lid_tightener}
            <br></br>
            <label>au41 Lid Droop:</label>
            <input type="range" 
                id='au41_lid_droop'
                value={this.v.au41_lid_droop} 
                min={-1}
                max={2}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au41_lid_droop}
            <br></br>
            <label>au42 Slit:</label>
            <input type="range" 
                id='au42_slit'
                value={this.v.au42_slit} 
                min={-1}
                max={2}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au42_slit}
            <br></br>
            <label>au43 Eyes Closed:</label>
            <input type="range" 
                id='au43_eyes_closed'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au43_eyes_closed} 
                onChange={this.handleChange} />
            {this.v.au43_eyes_closed}
            <br></br>
            <label>au44 Squint:</label>
            <input type="range" 
                id='au44_squint'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au44_squint} 
                onChange={this.handleChange} />
            {this.v.au44_squint}
            <br></br>
            <label>au45 Blink:</label>
            <input type="range" 
                id='au45_blink'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au45_blink} 
                onChange={this.handleChange} />
            {this.v.au45_blink}
            <br></br>
            <label>au61 Left:</label>
            <input type="range" 
                id='au61_left'
                value={this.v.au61_left} 
                min={-1}
                max={2}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au61_left}
            <br></br>
            <label>au62 Right:</label>
            <input type="range" 
                id='au62_right'
                value={this.v.au62_right} 
                min={-1}
                max={2}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au62_right}
            <br></br>
            <label>au63 Up:</label>
            <input type="range" 
                id='au63_up'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au63_up} 
                onChange={this.handleChange} />
            {this.v.au63_up}
            <br></br>
            <label>au64 Down:</label>
            <input type="range" 
                id='au64_down'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au64_down} 
                onChange={this.handleChange} />
            {this.v.au64_down}
            <br></br>
            <br></br>
        </form>
      );
    }
}

class BrowForm extends React.Component {
    constructor(props) {
        super(props);
        this.v = props.v
  
    this.handleChange = this.handleChange.bind(this);
    this.handleSubmit = this.handleSubmit.bind(this);
}
  
    handleChange(event) {
        let temp = {}
        temp[event.target.id] = event.target.value
        this.props.f(temp);
        this.v[event.target.id] = event.target.value
    }
  
    handleSubmit(event) {
      event.preventDefault();
    }
  
    render() {
      return (
        <form onSubmit={this.handleSubmit}>
          <label>Brows:</label>
            <br></br>
          <label>au1_ Inner Brow:</label>
            <input type="range" 
                id='au1_inner_brow_raiser'
                value={this.v.au1_inner_brow_raiser} 
                min={-1}
                max={2}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au1_inner_brow_raiser}
            <br></br>
            <label>au2_ Outer Brow:</label>
            <input type="range" 
                id='au2_outer_brow_raiser'
                value={this.v.au2_outer_brow_raiser} 
                min={-1}
                max={2}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au2_outer_brow_raiser}
            <br></br>
            <label>au4_ Brow Lower:</label>
            <input type="range" 
                id='au4_brow_lowerer'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au4_brow_lowerer} 
                onChange={this.handleChange} />
            {this.v.au4_brow_lowerer}
            <br></br>
            <br></br>
        </form>
      );
    }
}

class MouthForm extends React.Component {
    constructor(props) {
        super(props);
        this.v = props.v

        this.handleChange = this.handleChange.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleChange(event) {
        let temp = {}
        temp[event.target.id] = event.target.value
        this.props.f(temp);
        this.v[event.target.id] = event.target.value
    }

    handleSubmit(event) {
        event.preventDefault();
    }

    render() {
        return (
        <form onSubmit={this.handleSubmit}>
            <label>Lips:</label>
            <br></br>
            <label>au10 Raise Upper:</label>
            <input type="range" 
                id='au10_raise_upper'
                value={this.v.au10_raise_upper} 
                min={-1}
                max={2}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au10_raise_upper}
            <br></br>
            <label>au12 Lip Corners Out:</label>
            <input type="range" 
                id='au12_lip_corners_out'
                value={this.v.au12_lip_corners_out} 
                min={-1}
                max={2}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au12_lip_corners_out}
            <br></br>
            <label>au13 Cheek Puffer:</label>
            <input type="range" 
                id='au13_cheek_puffer'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au13_cheek_puffer} 
                onChange={this.handleChange} />
            {this.v.au13_cheek_puffer}
            <br></br>
            <label>au14 Dimpler:</label>
            <input type="range" 
                id='au14_dimpler'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au14_dimpler} 
                onChange={this.handleChange} />
            {this.v.au14_dimpler}
            <br></br>
            <label>au15 Lip Corner Depr:</label>
            <input type="range" 
                id='au15_lip_corner_depr'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au15_lip_corner_depr} 
                onChange={this.handleChange} />
            {this.v.au15_lip_corner_depr}
            <br></br>
            <label>au16 Lower Lip Depr:</label>
            <input type="range" 
                id='au16_lower_lip_depr'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au16_lower_lip_depr} 
                onChange={this.handleChange} />
            {this.v.au16_lower_lip_depr}
            <br></br>
            <label>au17 Chin Raiser:</label>
            <input type="range" 
                id='au17_chin_raiser'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au17_chin_raiser} 
                onChange={this.handleChange} />
            {this.v.au17_chin_raiser}
            <br></br>
            <label>au18 Lip Pucker:</label>
            <input type="range" 
                id='au18_lip_pucker'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au18_lip_pucker} 
                onChange={this.handleChange} />
            {this.v.au18_lip_pucker}
            <br></br>
            <label>au20 Lip Stretcher:</label>
            <input type="range" 
                id='au20_lip_stretcher'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au20_lip_stretcher} 
                onChange={this.handleChange} />
            {this.v.au20_lip_stretcher}
            <br></br>
            <label>au22 Lip Funnelerr:</label>
            <input type="range" 
                id='au22_lip_funneler'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au22_lip_funneler} 
                onChange={this.handleChange} />
            {this.v.au22_lip_funneler}
            <br></br>
            <label>au23 Lip Tightener:</label>
            <input type="range" 
                id='au23_lip_tightener'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au23_lip_tightener} 
                onChange={this.handleChange} />
            {this.v.au23_lip_tightener}
            <br></br>
            <label>au24 Lip Pressor:</label>
            <input type="range" 
                id='au24_lip_pressor'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au24_lip_pressor} 
                onChange={this.handleChange} />
            {this.v.au24_lip_pressor}
            <br></br>
            <label>au25 Lips Appart:</label>
            <input type="range" 
                id='au25_lips_part'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au25_lips_part} 
                onChange={this.handleChange} />
            {this.v.au25_lips_part}
            <br></br>
            <label>au26 Jaw Drop:</label>
            <input type="range" 
                id='au26_jaw_drop'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au26_jaw_drop} 
                onChange={this.handleChange} />
            {this.v.au26_jaw_drop}
            <br></br>
            <label>au27 Mouth Stretch:</label>
            <input type="range" 
                id='au27_mouth_stretch'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au27_mouth_stretch} 
                onChange={this.handleChange} />
            {this.v.au27_mouth_stretch}
            <br></br>
            <label>au28 Lip Suck:</label>
            <input type="range" 
                id='au28_lip_suck'
                min={-1}
                max={2}
                step={.03}
                value={this.v.au28_lip_suck} 
                onChange={this.handleChange} />
            {this.v.au28_lip_suck}
            <br></br>
        </form>
        );
    }
}

export {EyeForm, BrowForm, MouthForm}