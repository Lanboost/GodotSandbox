using Godot;
using System;

public partial class Main : Node2D
{
    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
        GD.Print("Hello world");
    }



    public void SmoothTerrain(float cordX, float cordY, int BrushSize, float Opacity)
    {
        var HeightData = new float[10, 10];
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
                    sectionsTotal += HeightData[x - 1, y];
                    adjacentSections++;

                    if ((y - 1) > 0) // Check up and to the left
                    {
                        sectionsTotal += HeightData[x - 1, y - 1];
                        adjacentSections++;
                    }

                    if ((y + 1) < cordY + BrushSize) // Check down and to the left
                    {
                        sectionsTotal += HeightData[x - 1, y + 1];
                        adjacentSections++;
                    }
                }

                if ((x + 1) < cordX + BrushSize) // Check to right
                {
                    sectionsTotal += HeightData[x + 1, y];
                    adjacentSections++;

                    if ((y - 1) > 0) // Check up and to the right
                    {
                        sectionsTotal += HeightData[x + 1, y - 1];
                        adjacentSections++;
                    }

                    if ((y + 1) < cordY + BrushSize) // Check down and to the right
                    {
                        sectionsTotal += HeightData[x + 1, y + 1];
                        adjacentSections++;
                    }
                }

                if ((y - 1) > 0) // Check above
                {
                    sectionsTotal += HeightData[x, y - 1];
                    adjacentSections++;
                }

                if ((y + 1) < cordY + BrushSize) // Check below
                {
                    sectionsTotal += HeightData[x, y + 1];
                    adjacentSections++;
                }

                newHeightData[x, y] = (HeightData[x, y] + (sectionsTotal / adjacentSections)) * Opacity;//0.5f;
            }
        }


        for (int x = (int)cordX - BrushSize; x <= cordX + BrushSize; x++)
        {
            for (int y = (int)cordY - BrushSize; y <= cordY + BrushSize; y++)

                HeightData[(int)cordX - BrushSize + x, (int)cordY - BrushSize + y] = newHeightData[x, y];

        }

    }
}