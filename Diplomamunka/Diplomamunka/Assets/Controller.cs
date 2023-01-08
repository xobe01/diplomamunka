using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Controller : MonoBehaviour
{
    [SerializeField] Material pointCloudSkybox;
    [SerializeField] Material objMaterial;
    [SerializeField] Color[] pointColors;

    Material defaultSkybox;
    LidarController lidarCont;
    MeshRenderer[] renderers;
    SkinnedMeshRenderer[] humanRenderers;
    List<GameObject> generatedObjects;
    bool renderersTurnedOff;
    bool processedShowed;
    bool displayedById;
    bool linesShowed;
    bool objectsShowed;

    void Start()
    {
        generatedObjects = new List<GameObject>();
        defaultSkybox = RenderSettings.skybox;
        lidarCont = FindObjectOfType<LidarController>();
        renderers = FindObjectsOfType<MeshRenderer>();
        humanRenderers = FindObjectsOfType<SkinnedMeshRenderer>();
        lidarCont.ReadProcessedData();
        lidarCont.Scan();
        ImportObjects();
        //lidarCont.Display(false);
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.X))
        {
            TurnOnOffRenderers(renderersTurnedOff);
            renderersTurnedOff = !renderersTurnedOff;
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            //lidarCont.Scan();
        }
        if (Input.GetKeyDown(KeyCode.P))
        {
            processedShowed = !processedShowed;
            lidarCont.Display(processedShowed);
        }
        if (Input.GetKeyDown(KeyCode.I))
        {
            displayedById = !displayedById;
            if (displayedById) lidarCont.DisplayById(0);
            else lidarCont.Display(processedShowed);
        }
        if (Input.GetKeyDown(KeyCode.L))
        {
            linesShowed = !linesShowed;
            lidarCont.DisplayLines(linesShowed);
        }
        if (Input.GetKeyDown(KeyCode.M))
        {
            objectsShowed = !objectsShowed;
            foreach (var obj in generatedObjects) obj.SetActive(objectsShowed);
        }
        if (displayedById)
        {
            if (Input.GetKeyDown(KeyCode.LeftArrow)) lidarCont.DisplayById(-1);
            if (Input.GetKeyDown(KeyCode.RightArrow)) lidarCont.DisplayById(1);
        }
    }

    void ImportObjects()
    {
        int counter = 0;
        GameObject obj = Resources.Load<GameObject>("processed_obj_0");
        while (obj != null)
        {
            var instance = (Instantiate(obj));
            instance.AddComponent<MeshFilter>();
            instance.GetComponent<MeshFilter>().mesh = obj.transform.GetChild(0)
                    .GetComponent<MeshFilter>().sharedMesh;
            instance.AddComponent<MeshRenderer>();
            Material m = Instantiate(objMaterial);
            m.color = pointColors[(counter + 1) % pointColors.Length];
            instance.GetComponent<MeshRenderer>().material = m;
            generatedObjects.Add(instance);
            instance.SetActive(false);
            counter++;
            obj = Resources.Load<GameObject>("processed_obj_" + counter);
        }
    }

    void TurnOnOffRenderers(bool on)
    {
        RenderSettings.skybox = on ? defaultSkybox : pointCloudSkybox;
        foreach (MeshRenderer m in renderers) m.enabled = on;
        foreach (SkinnedMeshRenderer m in humanRenderers) m.enabled = on;
    }

    public Color[] GetColors()
    {
        return pointColors;
    }
}
