function map_dictionary(D) {
    if (Array.isArray(D)) {
        return(
            D.map((key) => <option className="text-green-600" key={key} value={key}>{key}</option>)
        )
    }
    return (
        Object.keys(D).map((key) => <option className="text-blue-600" key={key} value={key}>{D[key]}</option>)
    )
}

function set_dropdown(label_text, value, update_function, options) {
    return (
        <>
        <label className="text-1xl font-bold">{label_text}:</label>
            <select id="test" className="text-xl "
                value={value} 
                multiple={false}
                onChange={(e) => update_function(e.target.value)}>
                {map_dictionary(options)}
            </select>
        
        </>
    )
}

export {set_dropdown, map_dictionary}