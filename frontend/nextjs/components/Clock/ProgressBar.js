function CircularProgressBar({ value, total, display }) {
    const fraction = value/total
    const percentage = fraction *100
    const radius = 40;
    const strokeWidth = 10;
    const circumference = 2 * Math.PI * radius;
    const offset = circumference - (percentage / 100) * circumference;
    var centerText = ""
    if (display.units === "%")
        { centerText = (100-percentage).toFixed(1).toString()+"%"}
    else {centerText = value+"%"}

    return (
        <svg width="100" height="100">
        <circle
            cx="50"
            cy="50"
            r={radius}
            strokeWidth={strokeWidth}
            fill="transparent"
            stroke="green"
        />
        <circle
            cx="50"
            cy="50"
            r={radius}
            strokeWidth={strokeWidth}
            fill="transparent"
            stroke="#f23a3a"
            strokeDasharray={circumference}
            strokeDashoffset={offset}
            transform="rotate(-90 50 50)"
        />
        <text x="50" y="50" textAnchor="middle" dominantBaseline="middle" fontSize="16">
            {centerText}
        </text>
        </svg>
    );
}

export default CircularProgressBar;
