# [AurixRacer_Second](./docs/index.md)

## 하드 구성
 * 하드는 3D 프린트를 통해 구성하였다.
   * Linescan Camera
    > TSL 1401 Linescan camera를 총 2개를 이용 총 3축으로 카메라의 위치를 조정할 수 있게 제작을 하여 최상의 카메라의 위치를 조정
    ![ㅇㅁ](https://user-images.githubusercontent.com/46989868/61260567-b3de7600-a7b9-11e9-9f49-8eb5a446e033.jpg)
   * Infrared ray sensor
    > 2Y0A02 적외선 센서 2개를 설치 _ 장애물의 인식율을 높이기 위하여 양쪽에 하나씩 설치하였다. 적외선의 높이를 능동적으로 조정하기 위하여 기둥 제작
    ![적외선](https://user-images.githubusercontent.com/46989868/61260709-3a935300-a7ba-11e9-909d-0b1f4a593c4a.jpg)
   * 전원부
    > 7.2V NiMH 배터리를 사용. 메인보드를 비롯하여 자동차의 많은 부품들이 5V를 정격 전원으로 이용하기에는 DC-DC Step down converter가 필요하다가 느낌. 자동차의 조향에 사용되는 servo 모터가 생각보다 많은 전류를 소모하였기에 통상적으로 많이 이용되는 7805 강압 컨버터 소자로는 부족하다고 느껴 정격 출력 전류가 높은 switching type의 컨버터를 구성하기로 함. 3A의 부하전류를 허용하는 LM2576-5를 이용하기로 결정.
   ![전원부](https://user-images.githubusercontent.com/46989868/61261139-bfcb3780-a7bb-11e9-88e1-83c870ed2a06.jpg)
   방열판을 차체의 외부에 흐르는 유체에 노출을 시켜 효율적으로 방열.
   
   
## 라인인식을 위한 알고리즘

* 처음에 하는 방식은 최적의 상태에서만 적용되는 알고리즘으로 환경의 요소가 조금이라도 추가 되면 라인을 인식하지 못하는 상황 발생
* 미분을 이용하여 이러한 문제 해결
* Line scan camera는 TSL1401을 사용 - 카메라를 통해 얻은 ADC data를 확인하면서 알고리즘 구성 주변의 환경에 비해 검은색 선 데이터가 
상대적으로 낮은 값인 것을 확인 할 수 있었음.
* 처음으로 뽑아낸 값에서 noise가 너무 많이 생겨 low pass filter을 적용
```c
if(idx == 0) {
    IR_LineScan.adcResult[chnIx][idx] = conversionResult.B.RESULT;
}
else {
    current = conversionResult.B.RESULT;
    previous = IR_LineScan.adcResult[chnIx][idx-1];
    value = current * 0.5 + previous * 0.5;
    IR_LineScan.adcResult[chnIx][idx] = value;
}
```

* low pass filter을 적용하여 얻은 그래프는 다음과 같다.
![원본](https://user-images.githubusercontent.com/46989868/61259770-99ef6400-a7b6-11e9-88b0-1488f1ba6081.png)

* 그 후에 조명과 주변 환경에도 영향을 덜 받기 위해서 미분한 기울기를 사용하기로 결정 검은선에서 급격하게 미분 기울기가 변할 것으로 예상 미분을 하여 찾은 그래프 형식은 다음과 같다.
![원본 + 미분](https://user-images.githubusercontent.com/46989868/61259848-f6528380-a7b6-11e9-858d-094db6a1c92f.png)

* 파형이 완만하지 못한 것을 확인할 수 있었음 그러므로 라인이 아닌 지점에서 기울기가 커지는 지점이 발생할 수 있으므로 threshold를 정해서 해당 값 이상은 그대로 두고, 나머지는 0으로 만들어 줌 이와 같은 방식으로 구성한 그래프는 다음과 같다.
![원본+미분(자름)](https://user-images.githubusercontent.com/46989868/61260005-94dee480-a7b7-11e9-8f42-67c60182d609.png)

```c
void makeDiff(int lr)
{
   for(int i = 0; i < DIFF_ARR_SIZE; i++)
   {
      diffArr[i] = ((float32)IR_LineScan.adcResult[lr][i + (WINDOW_SIZE - 1)] - (float32)IR_LineScan.adcResult[lr][i]) /            WINDOW_SIZE;
      diffArr[i] = (ABS(diffArr[i]) < DIFF_THRESHOLD) ? 0 : diffArr[i];
   }
}
```

* 원하는 그래프는 완만한 그래프 였으나 중간에 0인 것을 확인해 볼 수 있었다. 알파벳 N과 같은 형식으로 완만하게 이어주기 위하여 평균 filter를 통과 시켰다. 밑에 보이는 초록색 선이 그에 해당하는 결과물이다.
![원본+미분+필터](https://user-images.githubusercontent.com/46989868/61260094-e5eed880-a7b7-11e9-88f1-f0b6e0c02396.png)

```c
void averageFilter(void)
{
   int sum;
   for(int i = 0; i < FILTERED_ARR_SIZE; i++)
   {
      sum = 0;
      for(int j = 0; j < FILTER_SIZE; j++)
      {
         sum += diffArr[i + j];
      }
      filteredArr[i] = (float32)sum / FILTER_SIZE;
   }
}
```

* 위의 결과물을 다시 미분을 해주면 보라색 선을 보면 알듯이 정확하게 라인을 찾은 것을 알 수 있다. 이렇게 이중 미분을 통해 차선이라고 인식하게 하여 line detecting을 완료해주었다.
![원본+미분+필터+이중미분](https://user-images.githubusercontent.com/46989868/61260197-3b2aea00-a7b8-11e9-8dd7-5757afb48c5f.png)

```c
void makeTwoDiff(void)
{
   for(int i = 0; i < TWO_DIFF_ARR_SIZE; i++)
   {
      twoDiffArr[i] = (filteredArr[i + (WINDOW_SIZE - 1)] - filteredArr[i]) / WINDOW_SIZE;
   }
}
```
