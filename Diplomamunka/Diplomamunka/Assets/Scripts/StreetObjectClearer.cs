using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StreetObjectClearer : MonoBehaviour
{
    [SerializeField] bool DestroyRenderer;

    static int streetObjectIndex;
    public int index;

    void Awake()
    {
        index = streetObjectIndex;
        streetObjectIndex++;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.tag == "StreetObject" && other.GetComponent<StreetObjectClearer>().index < index)
        {
            Destroy(gameObject);
        }
    }

    public void TurnOffCollder()
    {
        Destroy(GetComponent<BoxCollider>());
        if(DestroyRenderer)
            Destroy(GetComponent<MeshRenderer>());
    }
}
