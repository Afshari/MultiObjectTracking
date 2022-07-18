import QtQuick 2.12
import QtQml 2.12
import QtQuick.Dialogs 1.1
import QtQuick.Controls 2.12
import "../controls"


Item {

    MessageDialog {
        id: messageDialog
        title: "Warning"
        icon: StandardIcon.Warning
        onAccepted: {
        }
    }


    QtObject {

        id:                                         tracking_params
        property variant points:                    []
        property variant clutters:                  []
        property variant measurements:              []
        property variant nearest_neighbor:          []
        property variant pda:                       []
        property variant gaussian_sum:              []
        property bool show_nearest_neighbor:        true
        property bool show_pda:                     true
        property bool show_gaussian_sum:            true
        property bool show_measurement:             true
        property bool show_all_meas:                true
        property bool show_truth:                   true
        property int  line_pointer:                 0
        property int  lambda_c:                     0
    }

    Timer {
        id: timer
        interval: 15;
        running: false;
        repeat: true
        onTriggered: {
            tracking_params.line_pointer += 1
            canvas.requestPaint()
        }
    }

    function sendMeasurement(measurements, clutters, lambda_c, curr_pointer) {

        var start_idx = parseInt(lambda_c * (curr_pointer-1));
        var end_idx = parseInt(lambda_c * curr_pointer);
        var curr_clutter = clutters.slice(start_idx, end_idx)
        var xs = [];
        var ys = [];
        for(var j = 0; j < curr_clutter.length; j++) {
            xs.push(curr_clutter[j][0]);
            ys.push(curr_clutter[j][1]);
        }
        xs.push(measurements[curr_pointer][0]);
        ys.push(measurements[curr_pointer][1]);
        backend.getMeasurements(xs, ys);
    }

    function drawPoints(ctx, items, color, radius=5, should_fill=false) {

        var x = 0;
        var y = 0;
        for(var i = 0; i < items.length; i++) {
            ctx.beginPath();
            if(should_fill === true)        ctx.fillStyle = color
            else                            ctx.strokeStyle = color
            x = items[i][0]
            y = items[i][1]
            ctx.arc(x, y, radius, 0, Math.PI * 2, false);
            if(should_fill === true)        ctx.fill();
            else                            ctx.stroke();
        }
    }

    function drawLine(ctx, points, curr_pointer) {

        var x = 0;
        var y = 0;
        for(var i = 0; i < curr_pointer; i++) {
            ctx.lineWidth = 3
            ctx.strokeStyle = canvas.color
            ctx.beginPath()
            x = points[i][0]
            y = points[i][1]
            ctx.moveTo(x, y)
            x = points[i + 1][0]
            y = points[i + 1][1]
            ctx.lineTo(x, y)
            ctx.stroke()
        }
    }

    function drawLegend(ctx, text_start_y, color, txt, TEXT_START_X, text_start_y, SYM_START_X) {

        ctx.beginPath();
        ctx.fillStyle = color;
        ctx.fillText(txt, TEXT_START_X, text_start_y);
        ctx.arc(SYM_START_X, text_start_y - 5, 5, 0, Math.PI * 2, false);
        ctx.fill()
    }

    function drawLegends(ctx) {

        ctx.beginPath();
        ctx.fillStyle = ctx.fillStyle = Qt.rgba(0.3, 0.3, 0.3, 1);
        ctx.rect(parent.width - 180, 20, 150, 170);
        ctx.fill();

        const TEXT_START_X = parent.width - 170;
        const SYM_START_X  = parent.width - 50;
        var text_start_y = 20;
        ctx.font = '16px Verdana'


        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.truth, "Truth", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.measurement, "Measure", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.nearest_neighbor, "NN", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.pda, "PDA", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.gaussian_sum, "GS", TEXT_START_X, text_start_y, SYM_START_X);
    }

    function shuffle(array) {

        let currentIndex = array.length,  randomIndex;

        while (currentIndex !== 0) {

            randomIndex = Math.floor(Math.random() * currentIndex);
            currentIndex--;

            [array[currentIndex], array[randomIndex]] = [array[randomIndex], array[currentIndex]];
        }

        return array;
    }

    function correctMeasurements(points) {

        while(true) {

            var found_gap = false;
            for(var i = 0; i < points.length-1; i++) {
                var x1 = points[i][0];
                var y1 = points[i][1];
                var x2 = points[i+1][0];
                var y2 = points[i+1][1];
                var d = Math.sqrt(Math.pow(x2-x1, 2) + Math.pow(y2-y1, 2));
                if(d >= 4) {
                    points.splice(i+1, 0, [(x1+x2)/2, (y1+y2)/2]);
                    found_gap = true;
                }
            }
            if(found_gap === false)
                break;
        }
        tracking_params.line_pointer = points.length;
        canvas.requestPaint()
    }

    QtObject {

        id:                                     state_machine
        readonly property int draw_line:        1
        readonly property int clutter:          2
        readonly property int timer_running:    3
        readonly property int timer_end:        4
        property int state:                     draw_line
    }

    Row {

        id: colorTools
        height: btnClear.height
        width: parent.width - 20
        spacing: 10
        anchors {
            horizontalCenter: parent.horizontalCenter
            top: parent.top
            topMargin: 8
        }
        property color truth:               "#33B5E5"
        property color nearest_neighbor:    "#FF0000"
        property color pda:                 "#00FF00"
        property color gaussian_sum:        "#0000FF"
        property color measurement:         "#FFFFFF"

        ButtonLabel {
            id: btnClear
            btnText: "erase"
            btnIconSource: "../../ui/images/svg_images/eraser.svg"
            onClicked: {

                if(state_machine.state === state_machine.timer_running) {
                    return;
                } else if(state_machine.state === state_machine.timer_end) {
                    state_machine.state = state_machine.draw_line;
                }

                tracking_params.points              = []
                tracking_params.clutters            = []
                tracking_params.measurements        = []
                tracking_params.nearest_neighbor    = []
                tracking_params.pda                 = []
                tracking_params.gaussian_sum        = []
                canvas.requestPaint()
            }
        }
        ButtonLabel {
            id: btnDrawLine
            btnText: "draw"
            btnIconSource: "../../ui/images/svg_images/draw_line.svg"
            onClicked: {
                if(state_machine.state === state_machine.clutter) {
                    state_machine.state = state_machine.draw_line
                }
            }
        }
        ButtonLabel {
            id: btnClutter
            btnText: "clutter"
            btnIconSource: "../../ui/images/svg_images/clutter.svg"
            onClicked: {
                if(state_machine.state == state_machine.draw_line) {
                    state_machine.state = state_machine.clutter
                }
            }
        }
        ButtonLabel {
            id: btnMeasurements
            btnText: "measure"
            btnIconSource: "../../ui/images/svg_images/measurement.svg"
            onClicked: {
                if(state_machine.state === state_machine.draw_line || state_machine.state === state_machine.clutter) {
                    if(tracking_params.points.length === 0) {
                        messageDialog.text = "First You have to Draw a Path";
                        messageDialog.visible = true;
                        return;
                    }

                    tracking_params.measurements = []
                    for(var i = 0; i < tracking_params.points.length; i++) {
                        var x = tracking_params.points[i][0] + (Math.random() - 0.5) * 15
                        var y = tracking_params.points[i][1] + (Math.random() - 0.5) * 15
                        tracking_params.measurements.push( [ parseInt(x), parseInt(y) ] )
                    }
                    canvas.requestPaint();
                }
            }
        }
        ButtonLabel {
            id: btnRun
            btnText: "Run"
            btnIconSource: "../../ui/images/svg_images/run.svg"
            onClicked: {

                if(tracking_params.points.length === 0) {
                    messageDialog.text = "There is no Data to Track";
                    messageDialog.visible = true;
                    return;
                }
                else if(tracking_params.measurements.length === 0) {
                    messageDialog.text = "There is no Measurement";
                    messageDialog.visible = true;
                    return;
                }
                else if(state_machine.state === state_machine.timer_running) {
                    return;
                }

                tracking_params.lambda_c = parseInt(tracking_params.clutters.length / tracking_params.measurements.length);
                backend.initSingleTrackers(tracking_params.points[0], tracking_params.points[1], tracking_params.lambda_c);
                if(timer.running === false) {
                    tracking_params.nearest_neighbor = []
                    tracking_params.pda              = []
                    tracking_params.gaussian_sum     = []
                    tracking_params.line_pointer = 0;
                    tracking_params.clutters = shuffle(tracking_params.clutters);
                    state_machine.state = state_machine.timer_running;
                    timer.start();
                }
            }
        }

        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "NN"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                tracking_params.show_nearest_neighbor = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "PDA"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                tracking_params.show_pda = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "GS"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                tracking_params.show_gaussian_sum = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "Measure"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                tracking_params.show_measurement = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "All Meas"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                tracking_params.show_all_meas = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "Truth"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                tracking_params.show_truth = checked
                canvas.requestPaint()
            }
        }

    }

    Canvas {
        id: canvas
        anchors {
            left:   parent.left
            right:  parent.right
            top:    colorTools.bottom
            bottom: parent.bottom
            margins: 8
        }
        property real lastX
        property real lastY
        property color color: colorTools.truth

        onPaint: {
            // console.log("onPaint")
            var ctx = getContext('2d')

            ctx.fillStyle = Qt.rgba(0.5, 0.5, 0.5, 1);
            ctx.fillRect(0, 0, width, height);

            var x = y = 0

            var curr_pointer = Math.min(tracking_params.line_pointer, tracking_params.points.length - 1);

            if(state_machine.state === state_machine.timer_running && tracking_params.line_pointer >= tracking_params.points.length) {
                timer.stop()
                state_machine.state = state_machine.timer_end;
            }

            if(tracking_params.show_truth === true) {

                drawLine(ctx, tracking_params.points, curr_pointer);
                //drawPoints(ctx, tracking_params.points, colorTools.truth);
            }

            if(tracking_params.show_nearest_neighbor === true)    drawPoints(ctx, tracking_params.nearest_neighbor, colorTools.nearest_neighbor)
            if(tracking_params.show_pda === true)                 drawPoints(ctx, tracking_params.pda, colorTools.pda)
            if(tracking_params.show_gaussian_sum === true)        drawPoints(ctx, tracking_params.gaussian_sum, colorTools.gaussian_sum)


            if(tracking_params.show_measurement == true) {

                var lambda_c = tracking_params.clutters.length / tracking_params.measurements.length;

                if(state_machine.state === state_machine.timer_running) {

                    sendMeasurement(tracking_params.measurements, tracking_params.clutters, lambda_c, curr_pointer)
                }


                if(tracking_params.show_all_meas == true) {

                    drawPoints(ctx, tracking_params.measurements.slice(0, curr_pointer), colorTools.measurement, 3, true)
                    drawPoints(ctx, tracking_params.clutters.slice(0, parseInt(lambda_c * curr_pointer)), colorTools.measurement, 3, true)

                } else {

                    if(state_machine.state === state_machine.timer_running || state_machine.state === state_machine.timer_end) {
                        if(curr_pointer > 0) {
                            drawPoints(ctx, tracking_params.clutters.slice(parseInt(lambda_c * (curr_pointer-1)),
                                                                           parseInt(lambda_c * curr_pointer)), colorTools.measurement, 3, true)
                            if(curr_pointer < tracking_params.measurements.length - 10)
                                drawPoints(ctx, tracking_params.measurements.slice(curr_pointer, curr_pointer+10), colorTools.measurement, 3, true)
                            else
                                drawPoints(ctx, tracking_params.measurements.slice(curr_pointer, tracking_params.measurements.length-1), colorTools.measurement, 3, true)
                        }
                    } else {
                        drawPoints(ctx, tracking_params.clutters, colorTools.measurement, 3, true)
                        drawPoints(ctx, tracking_params.measurements, colorTools.measurement, 3, true)
                    }
                }
            }

            drawLegends(ctx);
        }

        MouseArea {
            id: area
            anchors.fill: parent
            onPressed: {
                if(state_machine.state == state_machine.draw_line) {

                    tracking_params.clutters = []
                    tracking_params.measurements = []
                    tracking_params.points = []
                    tracking_params.points.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                    tracking_params.line_pointer = tracking_params.points.length

                } else if(state_machine.state == state_machine.clutter) {

                    var x = parseInt( mouseX )
                    var y = parseInt( mouseY )
                    for(var i = 0; i < 170; i++) {
                        var currX = x + (Math.random() - 0.5) * 400;
                        var currY = y + (Math.random() - 0.5) * 400;
                        tracking_params.clutters.push( [ currX, currY ] )
                    }
                    canvas.requestPaint()
                }
            }
            onReleased: {
                if(state_machine.state === state_machine.draw_line)
                    correctMeasurements(tracking_params.points)
            }
            onPositionChanged: {
                if(state_machine.state == state_machine.draw_line) {
                    tracking_params.points.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                    tracking_params.line_pointer = tracking_params.points.length
                }
                canvas.requestPaint()
            }
        }
    }

    Connections {

        target: backend

        onSingleTrackingAddItem: {
            if(typeOfItem.valueOf() === "nearest_neighbor") {
                tracking_params.nearest_neighbor.push( [ parseInt(x), parseInt(y) ] )
            } else if(typeOfItem.valueOf() === "pda") {
                tracking_params.pda.push( [ parseInt(x), parseInt(y) ] )
            } else if(typeOfItem.valueOf() === "gaussian_sum") {
                tracking_params.gaussian_sum.push( [ parseInt(x), parseInt(y) ] )
            } else if(typeOfItem.valueOf() === "repaint") {
                canvas.requestPaint()
            }
        }
    }

}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:800}
}
##^##*/
