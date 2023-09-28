using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrainDataGenerator : MonoBehaviour
{
    [SerializeField] List<GameObject> pedestrians;
    [SerializeField] List<GameObject> cars;
    [SerializeField] List<GameObject> houses;
    [SerializeField] List<GameObject> props;
    [SerializeField] List<GameObject> carSpawns;
    [SerializeField] List<GameObject> pedestrianSpawns;
    [SerializeField] GameObject crossingSpawn;
    [SerializeField] GameObject egoCar;

    List<GameObject> generatedObjects;
    LidarController lidarCont;

    void Awake()
    {
        lidarCont = FindObjectOfType<LidarController>();
        generatedObjects = new List<GameObject>();
    }

    public void GenerateSceneTrigger()
    {
        StartCoroutine(GenerateScene(1));
    }

    IEnumerator GenerateScene(int sceneCount)
    {
        for (int i = 0; i < sceneCount; i++)
        {
            foreach (var obj in generatedObjects)
            {
                Destroy(obj);
            }
            generatedObjects.Clear();
            Generate();
            yield return new WaitForSeconds(0.1f);
            print("start");
            foreach (var streetObjectClearer in FindObjectsOfType<StreetObjectClearer>())
            {
                streetObjectClearer.TurnOffCollder();
            }
            yield return new WaitForSeconds(1);
            lidarCont.Scan(false);
        }        
    }

     void Generate()
    {
        UnityEngine.Random.seed = 13;
        egoCar.transform.position = new Vector3(UnityEngine.Random.Range(0.0f, 1.0f) < 0.5f ? 0 : 7.5f, egoCar.transform.position.y,
            egoCar.transform.position.z);

        //houses
        for (int j = 0; j < 2; j++)
        {
            float houseDiff = 0;
            for (int i = 0; i < 20; i++)
            {
                var house = houses[UnityEngine.Random.Range(0, houses.Count)];
                float yRotation = 90 * UnityEngine.Random.Range(0, 4);
                houseDiff += (yRotation % 180 == 0 ? house.transform.Find("Cube").localScale.z :
                    house.transform.Find("Cube").localScale.x) / 2 + UnityEngine.Random.Range(-5, 5);
                Vector3 spawnPoint = new Vector3((j == 0 ? -10 : 33) + (yRotation % 180 == 0 ? house.transform.Find("Cube").localScale.x
                    : house.transform.Find("Cube").localScale.z) / 2 * (j == 0 ? -1 : 1) + UnityEngine.Random.Range(-3, 3), 0, -200
                    + houseDiff);
                houseDiff += (yRotation % 180 == 0 ? house.transform.Find("Cube").localScale.z : house.transform.Find("Cube").localScale.x) / 2;
                generatedObjects.Add(Instantiate(house, spawnPoint, Quaternion.Euler(house.transform.rotation.eulerAngles.x,
                    yRotation, house.transform.rotation.eulerAngles.z)));
            }
        }

        //cars
        foreach (var spawn in carSpawns)
        {
            int carCount = UnityEngine.Random.Range(10, 15);
            for (int j = 0; j < carCount; j++)
            {
                Vector3 spawnPoint = new Vector3(spawn.transform.position.x + UnityEngine.Random.Range(-spawn.transform.localScale.x * 0.3f,
                    spawn.transform.localScale.x * 0.3f), 0, spawn.transform.position.z +
                    UnityEngine.Random.Range(-spawn.transform.localScale.z / 2, spawn.transform.localScale.z / 2));
                var car = cars[UnityEngine.Random.Range(0, cars.Count)];
                generatedObjects.Add(Instantiate(car, spawnPoint, Quaternion.Euler(car.transform.rotation.eulerAngles.x, spawn.transform.rotation.eulerAngles.y,
                    car.transform.rotation.eulerAngles.z)));
            }            
        }
        //pedestrians + props
        foreach (var spawn in pedestrianSpawns)
        {
            int pedestrianCount = UnityEngine.Random.Range(10, 15);
            int propCount = UnityEngine.Random.Range(10, 20);
            for (int j = 0; j < pedestrianCount; j++)
            {
                Vector3 spawnPoint = new Vector3(spawn.transform.position.x + UnityEngine.Random.Range(-spawn.transform.localScale.x *
                    0.4f, spawn.transform.localScale.x * 0.4f), -0.6f, spawn.transform.position.z +
                    UnityEngine.Random.Range(-spawn.transform.localScale.z / 2, spawn.transform.localScale.z / 2));
                var pedestrian = pedestrians[UnityEngine.Random.Range(0, pedestrians.Count)];
                generatedObjects.Add(Instantiate(pedestrian, spawnPoint, Quaternion.Euler(pedestrian.transform.rotation.eulerAngles.x, 
                    UnityEngine.Random.Range(0,360), pedestrian.transform.rotation.eulerAngles.z)));
            }
            for (int j = 0; j < propCount; j++)
            {
                Vector3 spawnPoint = new Vector3(spawn.transform.position.x + UnityEngine.Random.Range(-spawn.transform.localScale.x *
                    0.3f, spawn.transform.localScale.x * 0.3f), -0.6f, spawn.transform.position.z +
                    UnityEngine.Random.Range(-spawn.transform.localScale.z / 2, spawn.transform.localScale.z / 2));
                var pedestrian = props[UnityEngine.Random.Range(0, props.Count)];
                generatedObjects.Add(Instantiate(pedestrian, spawnPoint, Quaternion.Euler(pedestrian.transform.rotation.eulerAngles.x,
                    UnityEngine.Random.Range(0, 360), pedestrian.transform.rotation.eulerAngles.z)));
            }
        }
        //crossing
        crossingSpawn.transform.position += Vector3.forward * UnityEngine.Random.Range(-200.0f, 200.0f);
        if(UnityEngine.Random.Range(0,1) < 0.6f)
        {
            int pedestrianCount = UnityEngine.Random.Range(1, 6);
            for (int j = 0; j < pedestrianCount; j++)
            {
                Vector3 spawnPoint = new Vector3(crossingSpawn.transform.position.x +
                UnityEngine.Random.Range(-crossingSpawn.transform.localScale.x *
                    0.4f, crossingSpawn.transform.localScale.x * 0.4f), -0.6f, crossingSpawn.transform.position.z +
                    UnityEngine.Random.Range(-crossingSpawn.transform.localScale.z / 2, crossingSpawn.transform.localScale.z / 2));
                var pedestrian = pedestrians[UnityEngine.Random.Range(0, pedestrians.Count)];
                generatedObjects.Add(Instantiate(pedestrian, spawnPoint, Quaternion.Euler(pedestrian.transform.rotation.eulerAngles.x,
                    UnityEngine.Random.Range(0, 360), pedestrian.transform.rotation.eulerAngles.z)));
            }
        }
        print("end");
    }
}
