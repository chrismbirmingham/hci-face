function map_dictionary(D) {
    if (Array.isArray(D)) {
        return(
            D.map((key) => <option key={key} value={key}>{key}</option>)
        )
    }
    return (
        Object.keys(D).map((key) => <option key={key} value={key}>{D[key]}</option>)
    )
}

function set_dropdown(label_text, value, update_function, options) {
    return (
        <label>{label_text}:
        <select value={value} 
            multiple={false}
            onChange={(e) => update_function(e.target.value)}>
            {map_dictionary(options)}
            </select>
        </label>
    )
}

export {set_dropdown, map_dictionary}