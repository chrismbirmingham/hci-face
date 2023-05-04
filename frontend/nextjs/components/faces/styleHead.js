


export default function StyleHead({face, head_settings}){
    var h = JSON.parse(JSON.stringify(head_settings))

    if (face === "mirror_on_the_wall"){
        h.positions.faceColor = h.eye_settings.skin = h.eye_settings.lid = "white" // "#D7E4F5"
        h.head_shape.image = "mirror-2.png"
        h.head_shape.shape_name = "mask"
    }
    if (face === "default"){
        h.head_shape.shape_name = "default"
        h.positions.faceColor = h.eye_settings.skin = h.eye_settings.lid = "#ffcb4c" // "#D7E4F5"
        h.head_shape.image = ""
        h.head_shape.concavity=1.2
        h.head_shape.head_width=45
        h.head_shape.y_offset=45
        h.positions.mouth.y = 25
        h.background_settings.width = 150
        h.background_settings.height = 150
        h.background_settings.y_0 = -80
    }
    if (face === "cordial"){
        h.positions.faceColor = h.eye_settings.skin = h.eye_settings.lid = "purple" // "#D7E4F5"
        h.head_shape.image = "mirror-2.png"
    }
    if (face === "qt"){
        h.head_shape.shape_name = "mask"
        h.positions.faceColor = h.eye_settings.skin = h.eye_settings.lid = "white" // "#D7E4F5"
        h.positions.right_eye.y = 0
        h.positions.left_eye.y = 0
        h.positions.right_brow.y = -7
        h.positions.left_brow.y = -7
        h.positions.mouth.y = 22

        h.head_shape.image = "qt.png"
        h.background_settings.width = 150
        h.background_settings.height = 150
        h.background_settings.y_0 = -80

        h.eye_settings.eye = "gray"
        h.eye_settings.iris_width = 1
        h.eye_shape.pupil_rad = 7
        h.eye_shape.xy_skew = .8
        h.eye_shape.boundary_rad = 12

        h.mouth_shape.nose_width = 0
        h.mouth_shape.nose_height = 0

        h.mouth_shape.thickness=0
        h.mouth_shape.width=10
        h.mouth_settings.lip_color="black"
        h.mouth_settings.chin_shading="0"

        h.brow_shape.ix=5
        h.brow_shape.ox=-12
    }
    if (face === "yoda"){
        h.head_shape.shape_name = "yoda"
        h.positions.faceColor = h.eye_settings.skin = h.eye_settings.lid = "green" // "#D7E4F5"
        h.head_shape.image = ""

        h.eye_settings.eye = "gray"
        h.eye_settings.iris_width = 1
        h.eye_shape.pupil_rad = 9
        h.eye_shape.xy_skew = 1.1
        h.eye_shape.boundary_rad = 12
        h.mouth_shape.thickness=2
        h.head_shape.head_width=150
        h.head_shape.head_height=90

        h.background_settings.width = 150
        h.background_settings.height = 150
        h.background_settings.y_0 = -80
    }
    return h
}