using Godot;
using System;
using System.Collections.Generic;

public partial class PointDrawer : Node
{
    [Export]
    public float scale;

    [Export]
    PackedScene pointPrefab;

    //List<Vector2> points = new List<Vector2>();
    public void ClearPoints()
    {
        while (this.GetChildCount() > 0) {
            this.RemoveChild(this.GetChild(0));
        }
    }

    public void AddPoint(Vector2 point, Color color)
    {
        //this.points.Add(point);
        var sprite = pointPrefab.Instantiate<Sprite2D>();
        this.AddChild(sprite);
        sprite.Position = new Vector2(point.X * scale, point.Y * scale);
        sprite.SelfModulate = color;
    }
}
