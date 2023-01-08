using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class LidarController : MonoBehaviour
{
    [SerializeField] GameObject PointSphere;
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
    List<int> verticalCount;
    List<GameObject> lines;
    Color[] pointColors;
    const float maxDistance = 200f;
    int verticalCounter = 0;
    int currentIdShowed = 0;
    int horizontalCount;

    void Awake()
    {
        cont = FindObjectOfType<Controller>();
        TurnOffColliders(false);
        processedPoints = new List<Point>();
        scannedPoints = new List<Point>();
        pointsById = new List<List<Point>>();
        verticalCount = new List<int>();
        lines = new List<GameObject>();
        particleSystem = GetComponent<ParticleSystem>();
        horizontalCount = pointCount / verticalPointCount;
        pointColors = cont.GetColors();
    }

    public void Scan()
    {
        TurnOffColliders(true);
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
        DisplayPoints(scannedPoints);
        SaveToFile();
        TurnOffColliders(false);
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
                    float.Parse(parts[2].Replace('.', ','))), int.Parse(parts[3]), int.Parse(parts[4]), int.Parse(parts[5]));
                processedPoints.Add(newPoint);
                int id = int.Parse(parts[5]);
                if (id > currentId)
                {
                    DrawLines(currentPointGroup, currentId);
                    currentId = id;
                    currentPointGroup.Clear();
                }
                //else if (id < currentId) throw new System.Exception("IDs not in order");
                currentPointGroup.Add(newPoint.Position);
                while (pointsById.Count < id + 1) pointsById.Add(new List<Point>());
                pointsById[id].Add(newPoint);
            }
        }
        DrawLines(currentPointGroup, currentId);
        reader.Close();
    }

    void ShootRay(Vector3 dir, int verticalIndex, int horizontalIndex)
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, dir, out hit))
        {
            if (hit.point.magnitude < maxDistance)
            {
                scannedPoints.Add(new Point(hit.point, verticalIndex, horizontalIndex, 0));
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
        lineRenderer.gameObject.SetActive(false);
        lines.Add(lineRenderer.gameObject);
    }

    public void DisplayLines(bool display)
    {
        foreach (var line in lines) line.SetActive(display);
    }

    public void Display(bool processed)
    {
        if (processed) DisplayPoints(processedPoints);
        else DisplayPoints(scannedPoints);
    }

    public void DisplayById(int step)
    {
        currentIdShowed += step;
        if (currentIdShowed == pointsById.Count) currentIdShowed = 0;
        else if (currentIdShowed == -1) currentIdShowed = pointsById.Count - 1;
        DisplayPoints(pointsById[currentIdShowed]);
    }

    void DisplayPoints(List<Point> points)//, Color[] colors)
    {
        cloud = new ParticleSystem.Particle[points.Count];
        for (int i = 0; i < points.Count; ++i)
        {
            cloud[i].position = points[i].Position;
            cloud[i].color = points[i].Id == 0 ? unprocesseedColor : pointColors[points[i].Id % pointColors.Length];
            cloud[i].size = .05f;
        }
        particleSystem.SetParticles(cloud, cloud.Length);
    }

    void SaveToFile()
    {
        string fileName = Application.dataPath + "/Resources/points_raw.txt";
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
