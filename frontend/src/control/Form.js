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
            <label>AU5 Upper Lid Raiser:</label>
            <input type="range" 
                id='au5'
                min={0}
                max={1}
                step={.03}
                value={this.v.au5} 
                onChange={this.handleChange} />
            {this.v.au5}
            <br></br>
            <label>AU6 Cheek Raiser:</label>
            <input type="range" 
                id='au6'
                min={0}
                max={1}
                step={.03}
                value={this.v.au6} 
                onChange={this.handleChange} />
            {this.v.au6}
            <br></br>
            <label>AU7 Lid Tightener:</label>
            <input type="range" 
                id='au7'
                min={0}
                max={1}
                step={.03}
                value={this.v.au7} 
                onChange={this.handleChange} />
            {this.v.au7}
            <br></br>
            <label>AU41 Lid Droop:</label>
            <input type="range" 
                id='au41'
                value={this.v.au41} 
                min={0}
                max={1}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au41}
            <br></br>
            <label>AU42 Slit:</label>
            <input type="range" 
                id='au42'
                value={this.v.au42} 
                min={0}
                max={1}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au42}
            <br></br>
            <label>AU43 Eyes Closed:</label>
            <input type="range" 
                id='au43'
                min={0}
                max={1}
                step={.03}
                value={this.v.au43} 
                onChange={this.handleChange} />
            {this.v.au43}
            <br></br>
            <label>AU44 Squint:</label>
            <input type="range" 
                id='au44'
                min={0}
                max={1}
                step={.03}
                value={this.v.au44} 
                onChange={this.handleChange} />
            {this.v.au44}
            <br></br>
            <label>AU45 Blink:</label>
            <input type="range" 
                id='au45'
                min={0}
                max={1}
                step={.03}
                value={this.v.au45} 
                onChange={this.handleChange} />
            {this.v.au45}
            <br></br>
            <label>AU61 Left:</label>
            <input type="range" 
                id='au61'
                value={this.v.au61} 
                min={0}
                max={1}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au61}
            <br></br>
            <label>AU62 Right:</label>
            <input type="range" 
                id='au62'
                value={this.v.au62} 
                min={0}
                max={1}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au62}
            <br></br>
            <label>AU63 Up:</label>
            <input type="range" 
                id='au63'
                min={0}
                max={1}
                step={.03}
                value={this.v.au63} 
                onChange={this.handleChange} />
            {this.v.au63}
            <br></br>
            <label>AU64 Down:</label>
            <input type="range" 
                id='au64'
                min={0}
                max={1}
                step={.03}
                value={this.v.au64} 
                onChange={this.handleChange} />
            {this.v.au64}
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
          <label>AU1 Inner Brow:</label>
            <input type="range" 
                id='au1'
                value={this.v.au1} 
                min={0}
                max={1}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au1}
            <br></br>
            <label>AU2 Outer Brow:</label>
            <input type="range" 
                id='au2'
                value={this.v.au2} 
                min={0}
                max={1}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au2}
            <br></br>
            <label>AU4 Brow Lower:</label>
            <input type="range" 
                id='au4'
                min={0}
                max={1}
                step={.03}
                value={this.v.au4} 
                onChange={this.handleChange} />
            {this.v.au4}
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
            <label>AU10 Raise Upper:</label>
            <input type="range" 
                id='au10'
                value={this.v.au10} 
                min={0}
                max={1}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au10}
            <br></br>
            <label>AU12 Lip Corners Out:</label>
            <input type="range" 
                id='au12'
                value={this.v.au12} 
                min={0}
                max={1}
                step={.03}
                onChange={this.handleChange} />
            {this.v.au12}
            <br></br>
            <label>AU13 Cheek Puffer:</label>
            <input type="range" 
                id='au13'
                min={0}
                max={1}
                step={.03}
                value={this.v.au13} 
                onChange={this.handleChange} />
            {this.v.au13}
            <br></br>
            <label>AU14 Dimpler:</label>
            <input type="range" 
                id='au14'
                min={0}
                max={1}
                step={.03}
                value={this.v.au14} 
                onChange={this.handleChange} />
            {this.v.au14}
            <br></br>
            <label>AU15 Lip Corner Depr:</label>
            <input type="range" 
                id='au15'
                min={0}
                max={1}
                step={.03}
                value={this.v.au15} 
                onChange={this.handleChange} />
            {this.v.au15}
            <br></br>
            <label>AU16 Lower Lip Depr:</label>
            <input type="range" 
                id='au16'
                min={0}
                max={1}
                step={.03}
                value={this.v.au16} 
                onChange={this.handleChange} />
            {this.v.au16}
            <br></br>
            <label>AU17 Chin Raiser:</label>
            <input type="range" 
                id='au17'
                min={0}
                max={1}
                step={.03}
                value={this.v.au17} 
                onChange={this.handleChange} />
            {this.v.au17}
            <br></br>
            <label>AU18 Lip Pucker:</label>
            <input type="range" 
                id='au18'
                min={0}
                max={1}
                step={.03}
                value={this.v.au18} 
                onChange={this.handleChange} />
            {this.v.au18}
            <br></br>
            <label>AU20 Lip Stretcher:</label>
            <input type="range" 
                id='au20'
                min={0}
                max={1}
                step={.03}
                value={this.v.au20} 
                onChange={this.handleChange} />
            {this.v.au20}
            <br></br>
            <label>AU22 Lip Funnelerr:</label>
            <input type="range" 
                id='au22'
                min={0}
                max={1}
                step={.03}
                value={this.v.au22} 
                onChange={this.handleChange} />
            {this.v.au22}
            <br></br>
            <label>AU23 Lip Tightener:</label>
            <input type="range" 
                id='au23'
                min={0}
                max={1}
                step={.03}
                value={this.v.au23} 
                onChange={this.handleChange} />
            {this.v.au23}
            <br></br>
            <label>AU24 Lip Pressor:</label>
            <input type="range" 
                id='au24'
                min={0}
                max={1}
                step={.03}
                value={this.v.au24} 
                onChange={this.handleChange} />
            {this.v.au24}
            <br></br>
            <label>AU25 Lips Appart:</label>
            <input type="range" 
                id='au25'
                min={0}
                max={1}
                step={.03}
                value={this.v.au25} 
                onChange={this.handleChange} />
            {this.v.au25}
            <br></br>
            <label>AU26 Jaw Drop:</label>
            <input type="range" 
                id='au26'
                min={0}
                max={1}
                step={.03}
                value={this.v.au26} 
                onChange={this.handleChange} />
            {this.v.au26}
            <br></br>
            <label>AU27 Mouth Stretch:</label>
            <input type="range" 
                id='au27'
                min={0}
                max={1}
                step={.03}
                value={this.v.au27} 
                onChange={this.handleChange} />
            {this.v.au27}
            <br></br>
            <label>AU28 Lip Suck:</label>
            <input type="range" 
                id='au28'
                min={0}
                max={1}
                step={.03}
                value={this.v.au28} 
                onChange={this.handleChange} />
            {this.v.au28}
            <br></br>
        </form>
        );
    }
}

export {EyeForm, BrowForm, MouthForm}