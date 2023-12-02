using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;

public class LidarController : MonoBehaviour
{
    [SerializeField] Vector2 angles;
    [SerializeField] int pointCount;
    [SerializeField] int verticalCount;
    [SerializeField] Color unprocesseedColor;
    [SerializeField] LineRenderer linePrefab;
    [SerializeField] GameObject normalArrow;

    Controller cont;
    ParticleSystem particleSystem;
    ParticleSystem.Particle[] cloud;
    List<Point> scannedPoints;
    List<Point> processedPoints;
    List<List<Point>> pointsById;
    List<List<Vector3>> linePoints;
    List<List<Vector3>> convexLinePoints;
    List<GameObject> lines;
    List<GameObject> convexLines;
    Color[] pointColors;
    const float maxDistance = 200f;
    int verticalCounter = 0;
    [SerializeField] int currentIdShowed = 0;
    int horizontalCount;

    int scenesGenerated = 0;
    bool showById = false;

    void Awake()
    {
        cont = FindObjectOfType<Controller>();
        processedPoints = new List<Point>();
        scannedPoints = new List<Point>();
        pointsById = new List<List<Point>>();
        linePoints = new List<List<Vector3>>();
        convexLinePoints= new List<List<Vector3>>();
        lines = new List<GameObject>();
        convexLines = new List<GameObject>();
        particleSystem = GetComponent<ParticleSystem>();
        horizontalCount = pointCount / verticalCount;
        if(cont != null)
            pointColors = cont.GetColors();
    }

    public void Scan(bool isGeneratorScene, bool isTest, int scanIndex)
    {
        scannedPoints.Clear();
        verticalCounter = 0;
        for (float i = 0; i < horizontalCount; i++)
        {
            for (int j = 0; j < verticalCount; j++)
            {
                ShootRay(new Vector3(Mathf.Sin(2 * Mathf.PI * (i / horizontalCount)),
                    angles.y / 45 + (angles.x - angles.y) / 45 / verticalCount * j ,
                    Mathf.Cos(2 * Mathf.PI * (i / horizontalCount))), j, (int)i); 
            }
        }
        if(scanIndex == -1)
            DisplayPoints(scannedPoints, 0);
        SaveToFile(isGeneratorScene, isTest, scanIndex);
        print("Scan done: " + scanIndex);
        if(!isGeneratorScene && scanIndex == -1) TurnOffColliders(false);
    }

    public void ReadRawData(int index)
    {
        string line;
        TextAsset data = Resources.Load<TextAsset>("points_raw_" + index);
        StreamReader reader = new StreamReader(new MemoryStream(data.bytes));
        line = reader.ReadLine();
        line = reader.ReadLine();
        while (reader.ReadLine() != line) { }
        while ((line = reader.ReadLine()) != null)
        {
            string[] parts = line.Split(';');
            scannedPoints.Add(new Point(new Vector3(float.Parse(parts[0]), float.Parse(parts[1]), float.Parse(parts[2])),0, 0, 0, 0, 0, 0, false));
        }
    }

    public void ReadProcessedData()
    {
        string line;
        TextAsset data = Resources.Load<TextAsset>("points_processed_test");
        StreamReader reader = new StreamReader(new MemoryStream(data.bytes));
        line = reader.ReadLine();
        int planeCount = int.Parse(line);
        for (int i = 0; i < planeCount; i++)
        {
            line = reader.ReadLine();
            string[] parts = line.Split(';');
            if(false)
                Instantiate(normalArrow, new Vector3(float.Parse(parts[0].Replace('.', ',')), float.Parse(parts[1].Replace('.', ',')),
                    float.Parse(parts[2].Replace('.', ','))), Quaternion.LookRotation(new Vector3(float.Parse(parts[3].Replace('.', ',')), 
                    float.Parse(parts[4].Replace('.', ',')), float.Parse(parts[5].Replace('.', ',')))));
        }
        List<Vector3> currentPointGroup = new List<Vector3>();
        while ((line = reader.ReadLine()) != null)
        {
            string[] parts = line.Split(';');
            if (parts.Length > 1)
            {
                Point newPoint = new Point(new Vector3(float.Parse(parts[0].Replace('.', ',')), float.Parse(parts[1].Replace('.', ',')),
                    float.Parse(parts[2].Replace('.', ','))), int.Parse(parts[3]), int.Parse(parts[4]), int.Parse(parts[5]),
                    int.Parse(parts[6]), int.Parse(parts[7]), 0, false);
                int id = int.Parse(parts[5]);
                int lineId = int.Parse(parts[7]);
                double temp = double.Parse(parts[8]);
                if (temp > 100000)
                    continue;
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
                for (int i = 0; i < (parts.Length - 9) / 2; i++)
                {
                    int convexLineId = int.Parse(parts[9 + i * 2]);
                    int convexLineIndex = int.Parse(parts[10 + i * 2]);
                    while (convexLinePoints.Count < convexLineId) convexLinePoints.Add(new List<Vector3>());
                    while (convexLinePoints[convexLineId - 1].Count < convexLineIndex + 1) convexLinePoints[convexLineId - 1].Add(Vector3.zero);
                    convexLinePoints[convexLineId - 1][convexLineIndex] = newPoint.Position;
                }
            }
        }
        for (int i = 0; i < linePoints.Count; i++)
        {
            if (linePoints[i].Count > 0)
                DrawLines(linePoints[i], i, false);
        }
        for (int i = 0; i < convexLinePoints.Count; i++)
        {
            if (convexLinePoints[i].Count > 0)
                DrawLines(convexLinePoints[i], i, true);
        }
        reader.Close();
    }

    void ShootRay(Vector3 dir, int verticalIndex, int horizontalIndex)
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, dir, out hit))
        {
            if (hit.point.magnitude < maxDistance && hit.collider.tag != "Ground")
            {
                scannedPoints.Add(new Point(hit.point, verticalIndex, horizontalIndex, 0, 0, 0, hit.point.magnitude, hit.collider.transform.root.tag != "StreetObject"));
                verticalCounter++;
            }
        }
    }

    void DrawLines(List<Vector3> points, int id, bool isConvexLine)
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
        (isConvexLine ? convexLines : lines).Add(lineRenderer.gameObject);
    }

    public void DisplayLines(bool display, bool isConvexLine)
    {
        foreach (var line in isConvexLine ? convexLines : lines) line.SetActive(display);
        foreach (var line in isConvexLine ? lines : convexLines) line.SetActive(false);
    }

    public void Display(bool processed, int type)
    {
        if (processed) DisplayPoints(showById ? pointsById[currentIdShowed] : processedPoints, type);
        else DisplayPoints(scannedPoints, 0);
    }

    public void DisplayById(int step, bool showById)
    {
        this.showById = showById;
        currentIdShowed += step;
        if (currentIdShowed == pointsById.Count) currentIdShowed = 0;
        else if (currentIdShowed == -1) currentIdShowed = pointsById.Count - 1;
        if (showById)
            DisplayPoints(pointsById[currentIdShowed], 0);
        else Display(true, 0);
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

    void SaveToFile(bool isGeneratorScene, bool isTest, int scanIndex)
    {
        string fileName = Application.dataPath + (isGeneratorScene ? "/Resources/Train_Data/" : "/Resources/points_raw_" +
            (scanIndex == -1 ? "test" : scanIndex) + ".txt");
        if (isGeneratorScene)
        {
            Texture2D texture = new Texture2D(horizontalCount, verticalCount);
            Texture2D textureLabel = new Texture2D(horizontalCount, verticalCount);
            UnityEngine.Color[] pixels = Enumerable.Repeat(UnityEngine.Color.black, horizontalCount * verticalCount).ToArray();
            texture.SetPixels(pixels);
            textureLabel.SetPixels(pixels);
            for (int i = 0; i < scannedPoints.Count; i++)
            {
                texture.SetPixel(scannedPoints[i].HorizontalIndex, -scannedPoints[i].VerticalIndex, new Color(1 - scannedPoints[i].Distance / maxDistance,
                   1 - scannedPoints[i].Distance / maxDistance, 1 - scannedPoints[i].Distance / maxDistance));
                if(!scannedPoints[i].IsHouse)
                    textureLabel.SetPixel(scannedPoints[i].HorizontalIndex, - scannedPoints[i].VerticalIndex, Color.white);
            }
            texture.Apply();
            textureLabel.Apply();
            scenesGenerated++;

            byte[] bytes = texture.EncodeToPNG();
            File.WriteAllBytes(fileName + (isTest ? "test" : "train") + "/points_" + scenesGenerated + ".png", bytes);
            bytes = textureLabel.EncodeToPNG();
            File.WriteAllBytes(fileName + (isTest ? "test" : "train") + "/points_" + scenesGenerated + "_label.png", bytes);
        }
        else
        {
            if (File.Exists(fileName))
            {
                File.Delete(fileName);
            }
            StreamWriter writer = new StreamWriter(fileName, true);
            writer.WriteLine(transform.position.x + ";" + transform.position.y + ";" + transform.position.z);
            writer.WriteLine(horizontalCount);
            writer.WriteLine(verticalCount);
            writer.WriteLine(scannedPoints.Count);
            for (int i = 0; i < scannedPoints.Count; i++)
            {
                writer.WriteLine(scannedPoints[i].Position.x + ";" + scannedPoints[i].Position.y + ";" + scannedPoints[i].Position.z + ';' + scannedPoints[i].HorizontalIndex
                    + ';' + scannedPoints[i].VerticalIndex + ';' + scannedPoints[i].Id);
            }
            writer.Close();
        }
    }

    void TurnOffColliders(bool on)
    {
        foreach (Collider c in FindObjectsOfType<Collider>()) c.enabled = on;
    }
}
