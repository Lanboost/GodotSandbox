using DeBroglie.Models;
using DeBroglie.Topo;
using DeBroglie;
using Godot;
using System;
using System.Collections.Generic;

public interface IChunk
{
	public int Size();

	public bool Blocked(int x, int y);
}

public class NavMeshRect
{
    public int sx;
    public int sy;
    public int ex;
    public int ey;

    public NavMeshRect(int sx, int sy, int ex, int ey)
    {
        this.sx = sx;
        this.sy = sy;
        this.ex = ex;
        this.ey = ey;
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
        return new NavMeshRect(sx, sy, ex, ey);
    }

    public void Create(IChunk chunk)
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
                    var rect = ExpandRect(used, y, x, size);
                    rects.Add(rect);
                }
            }
        }
	}
}


public partial class Main : Node2D
{
    float[][] HeightData;

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        GD.Print("Hello world");
        /*ITopoArray<char> sample = TopoArray.Create(new[]
        {
            //https://github.com/mxgmn/WaveFunctionCollapse
            //0 water, 1 grass, 2 mountain, 3 sand

            new[]{ '0', '0', '0'},
            new[]{ '1', '1', '1'},
            new[]{ '0', '2', '1'},

        }, periodic: false);
        // Specify the model used for generation

        foreach(var t in sample.ToTiles())
        {
            GD.Print(t);
        }

        var model = new AdjacentModel(sample.ToTiles());
        foreach(var tile in model.Tiles)
        {
            GD.Print(tile);
        }*/

        /*var model = new AdjacentModel(DirectionSet.Cartesian2d);
        var tile1 = new Tile(1);
        var tile2 = new Tile(2);
        var tile3 = new Tile(3);
        model.SetFrequency(tile1, 1);
        model.SetFrequency(tile2, 1);
        model.SetFrequency(tile3, 1);
        model.AddAdjacency(tile1, tile1, 1, 0, 0);
        model.AddAdjacency(tile1, tile1, 0, 1, 0);

        model.AddAdjacency(tile2, tile2, 1, 0, 0);
        model.AddAdjacency(tile2, tile2, 0, 1, 0);

        model.AddAdjacency(tile1, tile3, 1, 0, 0);
        model.AddAdjacency(tile1, tile3, 0, 1, 0);
        model.AddAdjacency(tile3, tile1, 1, 0, 0);
        model.AddAdjacency(tile3, tile1, 0, 1, 0);

        model.AddAdjacency(tile2, tile3, 1, 0, 0);
        model.AddAdjacency(tile2, tile3, 0, 1, 0);
        model.AddAdjacency(tile3, tile2, 1, 0, 0);
        model.AddAdjacency(tile3, tile2, 0, 1, 0);*/

        /*var a = new string[,]{
            {"w", "w"},
            {"w", "r"}
        };
        DeBroglie.Models.TileModel
        var model = AdjacentModel.Create(a, false);

        model.AddSample(
            TopoArray.Create(
                new[]{ 
                    new[]{ "w", "r", "w" },
                    new[]{ "w", "r", "g" },
                    new[]{ "w", "r", "g" },


                }, periodic: false).ToTiles());
        // Set the output dimensions
        var topology = new GridTopology(10, 10, periodic: false);
        // Acturally run the algorithm
        var propagator = new TilePropagator(model, topology);
        var status = propagator.Run();
        if (status != Resolution.Decided) throw new Exception("Undecided");
        var output = propagator.ToValueArray<string>();
        // Display the results
        for (var y = 0; y < 10; y++)
        {
            var str = "";
            for (var x = 0; x < 10; x++)
            {
                str += output.Get(x, y);
            }
            GD.Print(str);
        }*/
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
        UpdateMap();
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
        GD.Print("Hello world");
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
