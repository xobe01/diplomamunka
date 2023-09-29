using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class LidarController : MonoBehaviour
{
    [SerializeField] Vector2 angles;
    [SerializeField] int pointCount;
    [SerializeField] int verticalPointCount;
    [SerializeField] Color unprocesseedColor;
    [SerializeField] LineRenderer linePrefab;

    Controller cont;
    ParticleSystem particleSystem;
    ParticleSystem.Particle[] cloud;
    List<Point> scannedPoints;
    List<Point> processedPoints;
    List<List<Point>> pointsById;
    List<List<Vector3>> linePoints;
    List<int> verticalCount;
    List<GameObject> lines;
    Color[] pointColors;
    const float maxDistance = 200f;
    int verticalCounter = 0;
    int currentIdShowed = 0;
    int horizontalCount;

    int scenesGenerated = 0;

    void Awake()
    {
        cont = FindObjectOfType<Controller>();
        processedPoints = new List<Point>();
        scannedPoints = new List<Point>();
        pointsById = new List<List<Point>>();
        linePoints = new List<List<Vector3>>();
        verticalCount = new List<int>();
        lines = new List<GameObject>();
        particleSystem = GetComponent<ParticleSystem>();
        horizontalCount = pointCount / verticalPointCount;
        if(cont != null)
            pointColors = cont.GetColors();
    }

    public void Scan(bool isGeneratorScene)
    {
        verticalCount.Clear();
        scannedPoints.Clear();
        verticalCounter = 0;
        for (float i = 0; i < horizontalCount; i++)
        {
            for (int j = 0; j < verticalPointCount; j++)
            {
                ShootRay(new Vector3(Mathf.Sin(2 * Mathf.PI * (i / horizontalCount)),
                    angles.y / 45 + (angles.x - angles.y) / 45 / verticalPointCount * j ,
                    Mathf.Cos(2 * Mathf.PI * (i / horizontalCount))), j, (int)i); 
            }
            verticalCount.Add(verticalCounter);
        }
        DisplayPoints(scannedPoints, 0);
        SaveToFile(isGeneratorScene);
        if(!isGeneratorScene) TurnOffColliders(false);
    }

    public void ReadProcessedData()
    {
        string line;
        TextAsset data = Resources.Load<TextAsset>("points_processed");
        StreamReader reader = new StreamReader(new MemoryStream(data.bytes));
        int currentId = 1;
        List<Vector3> currentPointGroup = new List<Vector3>();
        while ((line = reader.ReadLine()) != null)
        {
            string[] parts = line.Split(';');
            if (parts.Length > 1)
            {
                Point newPoint = new Point(new Vector3(float.Parse(parts[0].Replace('.', ',')), float.Parse(parts[1].Replace('.', ',')),
                    float.Parse(parts[2].Replace('.', ','))), int.Parse(parts[3]), int.Parse(parts[4]), int.Parse(parts[5]),
                    int.Parse(parts[6]), int.Parse(parts[7]));
                int id = int.Parse(parts[5]);
                int lineId = int.Parse(parts[7]);
                int lineIndex = int.Parse(parts[8]);
                while (pointsById.Count < id + 1) pointsById.Add(new List<Point>());
                pointsById[id].Add(newPoint);
                processedPoints.Add(newPoint);
                while (linePoints.Count < lineId) linePoints.Add(new List<Vector3>());
                if (lineId != 0)
                {                    
                    while (linePoints[lineId - 1].Count < lineIndex + 1) linePoints[lineId - 1].Add(Vector3.zero);
                    linePoints[lineId - 1][lineIndex] = newPoint.Position;
                }                    
            }
        }
        for (int i = 0; i < linePoints.Count; i++)
        {
            print(linePoints[i].Count);
            DrawLines(linePoints[i], i);
        }
        reader.Close();
    }

    void ShootRay(Vector3 dir, int verticalIndex, int horizontalIndex)
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, dir, out hit))
        {
            if (hit.point.magnitude < maxDistance)
            {
                scannedPoints.Add(new Point(hit.point, verticalIndex, horizontalIndex, 0, 0, 0));
                verticalCounter++;
            }
        }
    }

    void DrawLines(List<Vector3> points, int id)
    {
        var lineRenderer = Instantiate(linePrefab).GetComponent<LineRenderer>();
        lineRenderer.startColor = pointColors[id % pointColors.Length];
        lineRenderer.endColor = pointColors[id % pointColors.Length];
        lineRenderer.positionCount = points.Count + 1;
        for (int i = 0; i < points.Count; i++)
        {
            lineRenderer.SetPosition(i, points[i]);
        }
        lineRenderer.SetPosition(points.Count, points[0]);
        lines.Add(lineRenderer.gameObject);
    }

    public void DisplayLines(bool display)
    {
        foreach (var line in lines) line.SetActive(display);
    }

    public void Display(bool processed, int type)
    {
        if (processed) DisplayPoints(processedPoints, type);
        else DisplayPoints(scannedPoints, 0);
    }

    public void DisplayById(int step)
    {
        currentIdShowed += step;
        if (currentIdShowed == pointsById.Count) currentIdShowed = 0;
        else if (currentIdShowed == -1) currentIdShowed = pointsById.Count - 1;
        DisplayPoints(pointsById[currentIdShowed], 0);
    }

    void DisplayPoints(List<Point> points, int type)
    {
        cloud = new ParticleSystem.Particle[points.Count];
        for (int i = 0; i < points.Count; ++i)
        {
            cloud[i].position = points[i].Position;
            if(type == 0)
                cloud[i].color = (points[i].Id == 0) ? unprocesseedColor : pointColors[points[i].Id % pointColors.Length];
            if (type == 1)
                cloud[i].color = points[i].OutlineId == 0 ? unprocesseedColor : pointColors[points[i].OutlineId % pointColors.Length];
            if (type == 2)
                cloud[i].color = points[i].LineId == 0 ? unprocesseedColor : pointColors[points[i].LineId % pointColors.Length];
            cloud[i].size = .05f;
        }
        particleSystem.SetParticles(cloud, cloud.Length);
    }

    void SaveToFile(bool isGeneratorScene)
    {
        string fileName = Application.dataPath + (isGeneratorScene ? "/Resources/Train_Data/points_"+ scenesGenerated + ".txt" : "/Resources/points_raw.txt");
        if (isGeneratorScene) scenesGenerated++;
        if (File.Exists(fileName))
        {
            File.Delete(fileName);
        }
        StreamWriter writer = new StreamWriter(fileName, true);
        writer.WriteLine(scannedPoints.Count);
        writer.WriteLine(horizontalCount);
        writer.WriteLine(verticalPointCount);
        for (int i = 0; i < verticalCount.Count; i++)
        {
            writer.WriteLine(verticalCount[i]);
        }
        for (int i = 0; i < scannedPoints.Count; i++){
            writer.WriteLine(scannedPoints[i].Position.x + ";" + scannedPoints[i].Position.y + ";" + scannedPoints[i].Position.z +';' + scannedPoints[i].HorizontalIndex
                + ';' + scannedPoints[i].VerticalIndex + ';' + scannedPoints[i].Id);
        }
        writer.Close();
    }

    void TurnOffColliders(bool on)
    {
        foreach (Collider c in FindObjectsOfType<Collider>()) c.enabled = on;
    }
}
