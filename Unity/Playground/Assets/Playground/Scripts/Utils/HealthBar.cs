using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class HealthBar : MonoBehaviour
{
    
    public Slider currentHealthBarSlider;
    public Slider easeHealthBarSlider;
    public float maxHealth = 100;
    public float currentHealth = 100;

    public float lerpSpeed = 0.02f;


    void Start()
    {
        currentHealth = maxHealth;
    }

    // Update is called once per frame
    void Update()
    {
        if(currentHealthBarSlider.value != currentHealth)
        {
            currentHealthBarSlider.value = currentHealth;
        }

        if(Input.GetKeyDown(KeyCode.Space))
        {
            TakeDamage(20);
        }

        if(currentHealthBarSlider.value != easeHealthBarSlider.value)
        {
            easeHealthBarSlider.value = Mathf.Lerp(easeHealthBarSlider.value, currentHealthBarSlider.value, lerpSpeed);
        }
    }

    void TakeDamage(float damage)
    {
        currentHealth -= damage;
        if(currentHealth <= 0)
        {
            currentHealth = 0;
        }
    }


}
