---
onenote-id: 0-24319ed99c514681832d8b83b5a2ed82!1-BC985D38C6EE9748!sfe254750458a4ca2a142f1061d7c5546
---
With default setup I just get 0s and 1s
   

```
else if (GPIO_Pin == GPIO_PIN_10) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ==
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
            right_encoder_count--;
        } 
        else {
            right_encoder_count++;
        }
    } 
    else if (GPIO_Pin == GPIO_PIN_8) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ==
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
            right_encoder_count++;
        } 
        else {
            right_encoder_count--;
        }
    }
```
 
With only GPIO 10, it doesn't seem to activate
   

```
else if (GPIO_Pin == GPIO_PIN_10) {
        // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ==
        //     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
        //     right_encoder_count--;
        // } 
        // else {
        //     right_encoder_count++;
        // }
        right_encoder_count++;
    } 
    else if (GPIO_Pin == GPIO_PIN_8) {
        // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ==
        //     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
        //     right_encoder_count++;
        // } 
        // else {
        //     right_encoder_count--;
        // }
    }
```
 
With only GPIO 8 it seems to be working
 
```
else if (GPIO_Pin == GPIO_PIN_10) {
        // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ==
        //     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
        //     right_encoder_count--;
        // } 
        // else {
        //     right_encoder_count++;
        // }
    } 
    else if (GPIO_Pin == GPIO_PIN_8) {
        // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ==
        //     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
        //     right_encoder_count++;
        // } 
        // else {
        //     right_encoder_count--;
        // }
        right_encoder_count++;
    }
```
 
2.875000,1  
2.880000,1  
2.885000,1  
2.890000,1  
2.895000,1  
2.900000,1  
2.905000,0  
2.910000,0  
2.915000,0  
2.920000,0  
2.925000,1  
2.930000,1  
2.935000,1

2.885000,0  
2.890000,0  
2.895000,0  
2.900000,0  
2.905000,0  
2.910000,0  
2.915000,0  
2.920000,0  
2.925000,0  
2.930000,0  
2.935000,0  
2.940000,0  
2.945000,0  
2.950000,0

2.920000,456  
2.925000,457  
2.930000,459  
2.935000,460  
2.940000,461  
2.945000,462  
2.950000,463  
2.955000,465  
2.960000,466  
2.965000,467  
2.970000,468  
2.975000,469  
2.980000,470  
2.985000,471  
2.990000,471  
2.995000,472