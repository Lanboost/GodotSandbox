using Godot;
using Lanboost.PathFinding.Astar;
using Lanboost.PathFinding.Graph;
using System;
using System.Collections.Generic;

public class NavMesh
{


}


public class NavMeshEdge
{
    public NavMeshRect left;
    public NavMeshRect right;
    public NavMeshRect edge;
    public float cost;

    public NavMeshEdge(NavMeshRect left, NavMeshRect right, NavMeshRect edge, float cost)
    {
        this.left = left;
        this.right = right;
        this.edge = edge;
        this.cost = cost;
    }
}

public class NavMeshRect
{
    public int sx;
    public int sy;
    public int width;
    public int height;

    public List<NavMeshEdge> edges = new List<NavMeshEdge>();

    public NavMeshRect(int sx, int sy, int width, int height)
    {
        this.sx = sx;
        this.sy = sy;
        this.width = width;
        this.height = height;
    }

    public float ManhattanDistance(NavMeshRect other)
    {
        var midx1 = this.sx + this.width / 2;
        var midy1 = this.sy + this.height / 2;

        var midx2 = other.sx + other.width / 2;
        var midy2 = other.sy + other.height / 2;

        return Math.Abs(midx1 - midx2) + Math.Abs(midy1 - midy2);
    }

    public NavMeshEdge CreateEdge(NavMeshRect other)
    {
        // check box intersect
        if (this.sx <= other.sx + other.width && other.sx <= this.sx + this.width)
        {
            if (this.sy <= other.sy + other.height && other.sy <= this.sy + this.height)
            {
                int sx, sy, ex, ey;
                // one side should line up with other
                if (this.sx == other.sx + other.width)
                {
                    sx = this.sx;
                    ex = this.sx;
                    sy = Math.Max(this.sy, other.sy);
                    ey = Math.Min(this.sy + this.height, other.sy + other.height);
                }
                else if (this.sx + this.width == other.sx)
                {
                    sx = other.sx;
                    ex = other.sx;
                    sy = Math.Max(this.sy, other.sy);
                    ey = Math.Min(this.sy + this.height, other.sy + other.height);
                }
                else if (this.sy == other.sy + other.height)
                {
                    sy = this.sy;
                    ey = this.sy;
                    sx = Math.Max(this.sx, other.sx);
                    ex = Math.Min(this.sx + this.width, other.sx + other.width);
                }
                else if (this.sy + this.height == other.sy)
                {
                    sy = other.sy;
                    ey = other.sy;
                    sx = Math.Max(this.sx, other.sx);
                    ex = Math.Min(this.sx + this.width, other.sx + other.width);
                }
                else
                {
                    throw new Exception("Wtf? Real overlap?");
                }

                return new NavMeshEdge(this, other, new NavMeshRect(sx, sy, ex - sx, ey - sy), ManhattanDistance(other));
            }
        }
        return null;
    }
}


public class NavMeshGraph : IGraph<NavMeshRect, NavMeshEdge>
{
    AStar<NavMeshRect, NavMeshEdge> astar;

    public List<NavMeshRect> navMeshRects = new List<NavMeshRect>();

    public NavMeshGraph(List<NavMeshRect> navMeshRects)
    {
        this.navMeshRects = navMeshRects;
        astar = new AStar<NavMeshRect, NavMeshEdge>(this, 100);
    }

    public NavMeshRect GetRectFromPosition(Vector2I position)
    {
        foreach (var rect in navMeshRects)
        {
            if (rect.sx <= position.X && position.X < rect.sx + rect.width)
            {
                if (rect.sy <= position.Y && position.Y < rect.sy + rect.height)
                {
                    return rect;
                }
            }
        }
        return null;
    }

    public void AddTemporaryStartEndNodes(NavMeshRect start, NavMeshRect end)
    {

    }

    public int GetCost(NavMeshRect from, NavMeshRect to, NavMeshEdge link)
    {
        return (int)link.cost;
    }

    public IEnumerable<Edge<NavMeshRect, NavMeshEdge>> GetEdges(NavMeshRect node)
    {
        foreach (var edge in node.edges)
        {
            var e = node != edge.left ? edge.left : edge.right;
            yield return new Edge<NavMeshRect, NavMeshEdge>(e, edge);
        }
    }

    public int GetEstimation(NavMeshRect from, NavMeshRect to)
    {
        return (int)from.ManhattanDistance(to);
    }

    internal List<NavMeshEdge> FindPath(Vector2I start, Vector2I end)
    {
        var s = GetRectFromPosition(start);
        var e = GetRectFromPosition(end);

        if(s == null || e == null)
        {
            return null;
        }

        var failure = this.astar.FindPath(s,e);
        if (failure == null)
        {
            return this.astar.GetPathLinks();
        }
        else
        {
            GD.Print(failure);
        }
        return null;
    }
}
