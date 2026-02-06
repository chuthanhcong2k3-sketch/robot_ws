var ROS2D = ROS2D || {};

ROS2D.Viewer = function(options) {
    var opts = options || {};
    this.width = opts.width || 800;
    this.height = opts.height || 800;
    this.divID = opts.divID;

    var div = document.getElementById(this.divID);
    this.scene = new createjs.Container();
    this.stage = new createjs.Stage(div);

    this.stage.addChild(this.scene);

    createjs.Ticker.framerate = 30;
    createjs.Ticker.addEventListener("tick", this.stage);
};

ROS2D.LaserScan = function(options) {
    var opts = options || {};
    this.ros = opts.ros;
    this.topic = opts.topic || '/scan';

    this.rootObject = opts.rootObject;

    var shape = new createjs.Shape();
    this.rootObject.addChild(shape);

    var that = this;

    var listener = new ROSLIB.Topic({
        ros: this.ros,
        name: this.topic,
        messageType: 'sensor_msgs/msg/LaserScan'
    });

    listener.subscribe(function(message) {
        shape.graphics.clear();
        shape.graphics.beginStroke("#00FF00");

        var angle = message.angle_min;
        for (var i = 0; i < message.ranges.length; i++) {
            var r = message.ranges[i];
            if (r < message.range_max && r > message.range_min) {
                var x = r * Math.cos(angle) * 50;
                var y = r * Math.sin(angle) * 50;
                shape.graphics.moveTo(0,0).lineTo(x,y);
            }
            angle += message.angle_increment;
        }
    });
};
