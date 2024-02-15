using Godot;
using Lanboost.PathFinding.Astar;
using Lanboost.PathFinding.Graph;
using System;
using System.Collections.Generic;

public struct NavigationPoint
{
    public float X;
    public float Y;
    public int Layer;

    public NavigationPoint(float x, float y, int layer)
    {
        X = x;
        Y = y;
        Layer = layer;
    }

    public static NavigationPoint NaN = new NavigationPoint(float.NaN, float.NaN, int.MaxValue);
}

public class NavMesh
{
    int chunkSize;

    /** This method will create a navmesh chunk, it will not link meshes towards neighbor chunks, 
     * you need to explicity call GenerateAllRemoteLinks / GenerateRemoteLinks.
     * 
     */
    public void GenerateChunk(int chunkX, int chunkY)
    {

    }

    public void GenerateAllRemoteLinks(int chunkX, int chunkY) { }
    public void GenerateRemoteLinks(int chunkX, int chunkY) { }

    public void ReadChunk(int chunkX, int chunkY) { }
    public void SaveChunk(int chunkX, int chunkY) { }

    public void AddChunk() { }

    public void RemoveChunk(int chunkX, int chunkY) { }

    public NavigationPoint GetClosestMeshPoint(Vector2 point, int layer)
    {
        return NavigationPoint.NaN;
    }

    public List<NavigationPoint> FindPath(Vector2 from, Vector2 to)
    {
        return null;
    }
}


public class NavMeshEdge
{
    public NavMeshRect to;

    // Left / right here, is determined by which direction the edge is going
    // relative to the triangulation in Simple Stupid Funnel algorithm
    public Vector2 left;
    public Vector2 right;
    public float cost;
    //used for remote edges
    public ulong chunkKey = ulong.MaxValue;
    public int toId;

    public NavMeshEdge(NavMeshRect to, Vector2 left, Vector2 right, float cost)
    {
        this.to = to;
        this.left = left;
        this.right = right;
        this.cost = cost;
    }
}

public class NavMeshRect
{
    public int sx;
    public int sy;
    public int width;
    public int height;
    public int layer = 0;
    // id is only for linking chunks together
    public int id;

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
                Vector2 left, right;

                int sx, sy, ex, ey;
                // one side should line up with other
                // this is to the right going <-
                if (this.sx == other.sx + other.width)
                {
                    var ytop = Math.Max(this.sy, other.sy);
                    var ybottom = Math.Min(this.sy + this.height, other.sy + other.height);
                    left = new Vector2(this.sx, ybottom);
                    right = new Vector2(this.sx, ytop);
                }
                // left ->
                else if (this.sx + this.width == other.sx)
                {
                    var ytop = Math.Max(this.sy, other.sy);
                    var ybottom = Math.Min(this.sy + this.height, other.sy + other.height);
                    left = new Vector2(other.sx, ytop);
                    right = new Vector2(other.sx, ybottom);
                }
                // bottom ^
                else if (this.sy == other.sy + other.height)
                {
                    var xleft = Math.Max(this.sx, other.sx);
                    var xright = Math.Min(this.sx + this.width, other.sx + other.width);
                    left = new Vector2(xleft, this.sy);
                    right = new Vector2(xright, this.sy);

                }
                // top v
                else if (this.sy + this.height == other.sy)
                {
                    var xleft = Math.Max(this.sx, other.sx);
                    var xright = Math.Min(this.sx + this.width, other.sx + other.width);
                    left = new Vector2(xright, other.sy);
                    right = new Vector2(xleft, other.sy);

                }
                else
                {
                    throw new Exception("Wtf? Real overlap?");
                }

                return new NavMeshEdge(other, left, right, ManhattanDistance(other));
            }
        }
        return null;
    }
}

public class NavMeshChunk
{
    public List<NavMeshRect> navMeshRects = new List<NavMeshRect>();
    public int x;
    public int y;
    public NavMeshChunk(int x, int y, List<NavMeshRect> navMeshRects)
    {
        this.navMeshRects = navMeshRects;
        this.x = x;
        this.y = y;
    }
}

public class NavMeshGraph : IGraph<NavMeshRect, NavMeshEdge>
{

    public static ulong PositionKey(int x, int y)
    {
        return (ulong) y >> 16 + x;
    }

    AStar<NavMeshRect, NavMeshEdge> astar;

    public Dictionary<ulong, NavMeshChunk> chunks = new Dictionary<ulong, NavMeshChunk>();


    public Dictionary<NavMeshRect, NavMeshEdge> extraEndEdges = new Dictionary<NavMeshRect, NavMeshEdge>();

    public NavMeshGraph()
    {
        astar = new AStar<NavMeshRect, NavMeshEdge>(this, 100);
    }

    void ConnectChunkLinks(NavMeshChunk from, NavMeshChunk to, ulong toKey)
    {
        foreach(var rects in from.navMeshRects)
        {
            foreach(var edge in rects.edges)
            {
                if(edge.chunkKey == toKey)
                {
                    edge.to = to.navMeshRects[edge.toId];
                }
            }
        }
    }

    void DisconnectChunkLinks(NavMeshChunk from, ulong toKey)
    {
        foreach (var rects in from.navMeshRects)
        {
            foreach (var edge in rects.edges)
            {
                if (edge.chunkKey == toKey)
                {
                    edge.to = null;
                }
            }
        }
    }

    public void LoadChunk(NavMeshChunk chunk)
    {
        var chunkKey = PositionKey(chunk.x, chunk.y);
        this.chunks.Add(chunkKey, chunk);
        //iterate over surrounding chunks, and add remote edges if 

        for(int y = -1; y <= 1; y++)
        {
            for (int x = -1; x <= 1; x++)
            {
                if(x == 0 && y==0)
                {
                    continue;
                }

                var key = PositionKey(chunk.x - x, chunk.y - y);
                if(chunks.ContainsKey(key))
                {
                    var other = chunks[key];
                    ConnectChunkLinks(chunk, other, key);
                    ConnectChunkLinks(other, chunk, chunkKey);
                }
            }
        }

    }

    public void UnloadChunk(int chunkX, int chunkY)
    {
        var chunkKey = PositionKey(chunkX, chunkY);
        this.chunks.Remove(chunkKey);
        //iterate over surrounding chunks, and remove remote edges if needed

        for (int y = -1; y <= 1; y++)
        {
            for (int x = -1; x <= 1; x++)
            {
                if (x == 0 && y == 0)
                {
                    continue;
                }

                var key = PositionKey(chunkX - x, chunkY - y);
                if (chunks.ContainsKey(key))
                {
                    var other = chunks[key];
                    DisconnectChunkLinks(other, chunkKey);
                }
            }
        }
    }

    public NavMeshRect GetRectFromPosition(Vector2 position, int layer)
    {
        var key = PositionKey((int)position.X, (int)position.Y);

        if(chunks.ContainsKey(key))
        {
            foreach (var rect in chunks[key].navMeshRects)
            {
                if (rect.sx <= position.X && position.X < rect.sx + rect.width)
                {
                    if (rect.sy <= position.Y && position.Y < rect.sy + rect.height)
                    {
                        if(rect.layer ==  layer)
                        {
                            return rect;
                        }
                    }
                }
            }
        }
        return null;
    }

    public void AddTemporaryStartEndNodes(NavMeshRect start, NavMeshRect end)
    {
        var s = GetRectFromPosition(new Vector2(start.sx, start.sy), start.layer);
        var e = GetRectFromPosition(new Vector2(end.sx, end.sy), end.layer);

        if (s == null || e == null)
        {
            //return null;
        }
        else
        {
            if (s == e)
            {
                //start.edges.Add(new NavMeshEdge(end, se.left, se.right, 0));
                throw new Exception("Shoudl ensure this doesnt happen...Lan...");
            }
            else
            {

                foreach (var se in s.edges)
                {
                    var cost = start.ManhattanDistance(se.to);
                    start.edges.Add(new NavMeshEdge(se.to, se.left, se.right, cost));
                }

                extraEndEdges.Clear();
                foreach (var ee in e.edges)
                {
                    var cost = end.ManhattanDistance(ee.to);
                    extraEndEdges.Add(ee.to, new NavMeshEdge(end, ee.right, ee.left, cost));
                }
            }
        }
    }

    public int GetCost(NavMeshRect from, NavMeshRect to, NavMeshEdge link)
    {
        return (int)link.cost;
    }

    public IEnumerable<Edge<NavMeshRect, NavMeshEdge>> GetEdges(NavMeshRect node)
    {
        foreach (var edge in node.edges)
        {
            if(edge.to == null)
            {
                continue;
            }
            //var e = node != edge.left ? edge.left : edge.right;
            yield return new Edge<NavMeshRect, NavMeshEdge>(edge.to, edge);
        }
        if (extraEndEdges.ContainsKey(node))
        {
            var extra = extraEndEdges[node];
            if (extra.to != null)
            {
                yield return new Edge<NavMeshRect, NavMeshEdge>(extra.to, extra);
            }
        }
    }

    public int GetEstimation(NavMeshRect from, NavMeshRect to)
    {
        return (int)from.ManhattanDistance(to);
    }

    internal List<NavMeshEdge> FindPath(Vector2I start, Vector2I end)
    {
        

        var failure = this.astar.FindPath(new NavMeshRect(start.X, start.Y, 0, 0), new NavMeshRect(end.X, end.Y, 0,0));
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
