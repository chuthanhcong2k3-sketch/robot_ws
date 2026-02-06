var ROS2D = ROS2D || {
    REVISION: "0.9.0"
};
(function() {
    ROS2D.Grid = function(options) {
        var options = options || {};
        this.cellSize = options.cellSize || 10;
        createjs.Shape.call(this);
        this.setStrokeStyle(0.2);
        this.beginStroke(createjs.Graphics.getRGB(255, 255, 255, 0.3));
        for (var x = 0; x < options.width; x += this.cellSize) {
            this.moveTo(x, 0);
            this.lineTo(x, options.height);
        }
        for (var y = 0; y < options.height; y += this.cellSize) {
            this.moveTo(0, y);
            this.lineTo(options.width, y);
        }
    };
    ROS2D.Grid.prototype = Object.create(createjs.Shape.prototype);
})();
(function() {
    ROS2D.Viewer = function(options) {
        options = options || {};
        var divID = options.divID;
        var width = options.width || 800;
        var height = options.height || 600;
        this.scene = new createjs.Container();
        this.canvas = document.getElementById(divID);
        this.stage = new createjs.Stage(this.canvas);
        this.stage.addChild(this.scene);
        this.scene.scaleX = 1;
        this.scene.scaleY = -1;
        this.scene.y = height;
        createjs.Ticker.addEventListener("tick", this.stage);
    };
})();
(function() {
    ROS2D.LaserScan = function(options) {
        var that = this;
        options = options || {};
        var ros = options.ros;
        var topic = options.topic || '/scan';
        var rootObject = options.rootObject;

        this.graphics = new createjs.Shape();
        rootObject.addChild(this.graphics);

        this.sub = new ROSLIB.Topic({
            ros: ros,
            name: topic,
            messageType: 'sensor_msgs/msg/LaserScan'
        });

        this.sub.subscribe(function(msg) {
            that.graphics.graphics.clear();
            that.graphics.graphics.beginFill(createjs.Graphics.getRGB(255,0,0));

            var angle = msg.angle_min;
            for (var i = 0; i < msg.ranges.length; i++) {
                var r = msg.ranges[i];
                if (r < msg.range_max && r > msg.range_min) {
                    var x = r * Math.cos(angle);
                    var y = r * Math.sin(angle);
                    that.graphics.graphics.drawRect(x, y, 0.5, 0.5);
                }
                angle += msg.angle_increment;
            }
            that.graphics.graphics.endFill();
        });
    };
})();
