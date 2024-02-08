using DeBroglie.Models;
using DeBroglie.Topo;
using DeBroglie;
using Godot;
using System;
using System.Collections.Generic;
using DeBroglie.Rot;

public interface IChunk
{
	public int Size();

	public bool Blocked(int x, int y);
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

        return Math.Abs(midx1-midx2)+Math.Abs(midy1-midy2);
    }

    public NavMeshEdge CreateEdge(NavMeshRect other)
    {
        // check box intersect
        if(this.sx <= other.sx+other.width && other.sx <= this.sx+this.width)
        {
            if (this.sy <= other.sy+other.height && other.sy <= this.sy+this.height)
            {
                int sx, sy, ex, ey;
                // one side should line up with other
                if(this.sx == other.sx+other.width)
                {
                    sx = this.sx;
                    ex = this.sx;
                    sy = Math.Max(this.sy, other.sy);
                    ey = Math.Min(this.sy + this.height, other.sy + other.height);
                }
                else if (this.sx+this.width == other.sx)
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
                else if (this.sy+this.height == other.sy)
                {
                    sy = other.sy;
                    ey = other.sy;
                    sx = Math.Max(this.sx, other.sx);
                    ex = Math.Min(this.sx+this.width, other.sx+other.width);
                }
                else
                {
                    throw new Exception("Wtf? Real overlap?");
                }

                return new NavMeshEdge(this, other, new NavMeshRect(sx, sy, ex-sx, ey-sy), ManhattanDistance(other));
            }
        }
        return null;
    }
}
public class NavMeshCreator
{


    NavMeshRect ExpandRect(bool[][] usedCurrent, int sx, int sy, int size)
	{
		int ex = sx;
		int ey = sy;
		while(ex < size-1)
		{
			if (!usedCurrent[sy][ex+1])
			{
                ex ++;
				usedCurrent[sy][ex] = true;
            }
			else
			{
				break;
			}
		}

        while (ey < size - 1)
        {
            var ok = true;
            for(int x = sx; x <= ex; x++)
            {
                if (usedCurrent[ey+1][x])
                {
                    ok = false;
                    break;
                }
            }

            if(ok)
            {
                ey++;
                for (int x = sx; x <= ex; x++)
                {
                    usedCurrent[ey][x] = true;
                }
            }
            else
            {
                break;
            }
        }
        return new NavMeshRect(sx, sy, ex-sx+1, ey-sy+1);
    }

    public List<NavMeshRect> Create(IChunk chunk)
	{
        List<NavMeshRect> rects = new List<NavMeshRect>();
		var size = chunk.Size();

        bool[][] used = new bool[size][];
        for (int y = 0; y < used.Length; y++)
        {
			used[y] = new bool[size];
            for (int x = 0; x < used[y].Length; x++)
            {
				used[y][x] = chunk.Blocked(x, y);
            }
        }

        for (int y = 0; y < used.Length; y++)
		{
            for (int x = 0; x < used[y].Length; x++)
            {
				if (!used[y][x])
				{
                    var rect = ExpandRect(used, x, y, size);
                    rects.Add(rect);
                }
            }
        }
        return rects;
    }

    public List<NavMeshEdge> CreateEdges(List<NavMeshRect> rects)
    {
        List<NavMeshEdge> edges = new List<NavMeshEdge>();
        for (int curr = 0; curr < rects.Count; curr++)
        {
            for (int next = curr+1; next < rects.Count; next++)
            {
                // Check overlap / create edge
                var edge = rects[curr].CreateEdge(rects[next]);
                if(edge != null)
                {
                    edges.Add(edge);
                }
            }
        }
        return edges;
    }
}

public class TestChunk:IChunk
{
    public int[][] layout;

    public TestChunk(int[][] layout)
    {
        this.layout = layout;
    }

    public bool Blocked(int x, int y)
    {
        return layout[y][x] == 0;
    }

    public int Size()
    {
        return layout.Length;
    }
}

public partial class Main : Node2D
{
    float[][] HeightData;

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        GD.Print("Hello world");
        ITopoArray<char> sample = TopoArray.Create(new[]
        {
            //https://github.com/mxgmn/WaveFunctionCollapse
            //0 water, 1 grass, 2 mountain, 3 sand


            new[] { '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0'},
            new[] { '0', '0', '0', '1', '0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '1', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1'},
            new[] { '0', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '0', '1', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0'},
            new[] { '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0'},
            new[] { '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '1', '1', '1', '1', '0', '0'},
            new[] { '1', '1', '1', '1', '1', '1', '1', '1', '0', '0', '1', '1', '1', '1', '1', '1'},
            new[] { '0', '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0', '0'},
            new[] { '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '1', '1', '1', '1', '0', '0'},
            new[] { '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '0', '0', '1', '0', '0', '0'},
            new[] { '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0'}


        }, periodic: false);
        // Specify the model used for generation
        var model = new OverlappingModel(sample.ToTiles(), 3, 4, true);
        var msize = 20;
        // Set the output dimensions
        var topology = new GridTopology(msize, msize, periodic: false);
        // Acturally run the algorithm
        var propagator = new TilePropagator(model, topology);

        var status = propagator.Run();
        if (status != Resolution.Decided) throw new Exception("Undecided");
        var output = propagator.ToValueArray<char>();
        // Display the results
        for (var y = 0; y < msize; y++)
        {
            var str = "";
            for (var x = 0; x < msize; x++)
            {
                str += output.Get(x, y);
                if (output.Get(x, y) == '0')
                {
                    tilemap.SetCell(0, new Vector2I(x, y), 3, new Vector2I(0, 0));
                }else
                {
                    tilemap.SetCell(0, new Vector2I(x, y), 3, new Vector2I(2, 0));
                }
            }
            GD.Print(str);
        }


        var size = 10;
        HeightData = new float[size][];
        for (int y = 0; y < size; y++)
        {
            HeightData[y] = new float[size];
            for (int x = 0; x < size; x++)
            {
                HeightData[y][x] = y*size+x+1;
                //tilemap.SetCell(0, new Vector2I(x, y), 2, new Vector2I(x, y));
            }
        }
        //UpdateMap();

        var data = new int[msize][];
        for (var y = 0; y < msize; y++)
        {
            data[y] = new int[msize];
            for (var x = 0; x < msize; x++)
            {
                if (output.Get(x, y) == '0')
                {
                    data[y][x] = 0;
                }
                else
                {
                    data[y][x] = 1;
                }
            }
        }


        var tc = new TestChunk(data);
        var nc = new NavMeshCreator();
        var l = nc.Create(tc);
        var edges = nc.CreateEdges(l);

        var i = 0;
        foreach(var rect in l)
        {
            for(var cy = rect.sy; cy < rect.sy+rect.height; cy++)
            {
                for (var cx = rect.sx; cx < rect.sx + rect.width; cx++)
                {
                    tilemap.SetCell(1, new Vector2I(cx, cy), 2, new Vector2I(i % 10, i / 10));
                }
            }
            i += 1;
            i = i % 100;
        }

        foreach (var e in edges)
        {
            var ed = e.edge;
            if(ed.width == 0)
            {
                for(int y = ed.sy; y < ed.sy+ed.height; y++)
                {
                    tilemap.SetCell(2, new Vector2I(ed.sx, y), 3, new Vector2I(7, 0));
                }
            }
            else
            {
                for (int x = ed.sx; x < ed.sx+ed.width; x++)
                {
                    tilemap.SetCell(3, new Vector2I(x, ed.sy), 3, new Vector2I(4, 0));
                }
            }
        }

    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
    }

    public override void _UnhandledKeyInput(InputEvent @event)
    {
        base._UnhandledKeyInput(@event);
        SmoothTerrain(5, 5, 2, 1);
        UpdateMap();
    }

    void SetCell(int x, int y, int value)
    {
        var v = value - 1;
        var vy = (int) v / 10;
        var vx = v % 10;
        tilemap.SetCell(0, new Vector2I(x, y), 2, new Vector2I(vx, vy));
    }

    void UpdateMap()
    {


        for (int y = 0; y < HeightData.Length; y++)
        {
            for (int x = 0; x < HeightData[y].Length; x++)
            {
                SetCell(x, y, (int)HeightData[y][x]);
            }
        }
    }

    public void SmoothTerrain1(float cordX, float cordY, int BrushSize, float Opacity)
    {
        var markOffset = new int[][]
        {
            new int[] { 1, 0},
            new int[] { 1, 1},
            new int[] { 0, 1},
            new int[] {-1, 1},
            new int[] {-1, 0},
            new int[] {-1,-1},
            new int[] { 0,-1},
            new int[] { 1,-1},
        };
        
        // TODO iterate over all markOffsets, and combine value etc
    }

    public void SmoothTerrain(float cordX, float cordY, int BrushSize, float Opacity)
    {
        float[,] newHeightData;

        // Note: MapWidth and MapHeight should be equal and power-of-two values 
        newHeightData = new float[(BrushSize * 2) + 1, (BrushSize * 2) + 1];

        for (int x = (int)cordX - BrushSize; x <= cordX + BrushSize; x++)
        {
            for (int y = (int)cordY - BrushSize; y <= cordY + BrushSize; y++)
            {
                int adjacentSections = 0;
                float sectionsTotal = 0.0f;

                if ((x - 1) > 0) // Check to left
                {
                    sectionsTotal += HeightData[y][x - 1];
                    adjacentSections++;

                    if ((y - 1) > 0) // Check up and to the left
                    {
                        sectionsTotal += HeightData[y-1][x - 1];
                        adjacentSections++;
                    }

                    if ((y + 1) < cordY + BrushSize) // Check down and to the left
                    {
                        sectionsTotal += HeightData[y+1][x - 1];
                        adjacentSections++;
                    }
                }

                if ((x + 1) < cordX + BrushSize) // Check to right
                {
                    sectionsTotal += HeightData[y][x + 1];
                    adjacentSections++;

                    if ((y - 1) > 0) // Check up and to the right
                    {
                        sectionsTotal += HeightData[y-1][x + 1];
                        adjacentSections++;
                    }

                    if ((y + 1) < cordY + BrushSize) // Check down and to the right
                    {
                        sectionsTotal += HeightData[y+1][x + 1];
                        adjacentSections++;
                    }
                }

                if ((y - 1) > 0) // Check above
                {
                    sectionsTotal += HeightData[y-1][x];
                    adjacentSections++;
                }

                if ((y + 1) < cordY + BrushSize) // Check below
                {
                    sectionsTotal += HeightData[y+1][x];
                    adjacentSections++;
                }

                newHeightData[y, x] = (HeightData[y][x] + (sectionsTotal / adjacentSections)) * Opacity;//0.5f;
            }
        }


        for (int x = (int)cordX - BrushSize; x <= cordX + BrushSize; x++)
        {
            for (int y = (int)cordY - BrushSize; y <= cordY + BrushSize; y++)

                HeightData[(int)cordY - BrushSize + y][(int)cordX - BrushSize + x] = newHeightData[y, x];

        }

    }

    [Export]
    public TileMap tilemap;
}
