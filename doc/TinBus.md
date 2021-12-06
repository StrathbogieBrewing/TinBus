# Tinbus



## Differential Pulse
```wavedrom
{


  signal: [
  { name: "4 MHz", wave: 'P........' },
  {},
      {name: 'sig', wave: 'aaaaaaa', data: '0 0.7 1 0.7 0 -0.7 -1'},
  {name: 'D+', wave: 'z01..0..z' },
  {name: "D-", wave: 'z0..1..0z'},
  {name: "Vo", wave: 'z.u.zd.z.'}
]


}



```
# Flow Flowchart
```mermaid
flowchart TD
  A[Init] --> B[RX Idle]

  B -- Yes --> C[OK];
  C --> D[Rethink];
  D --> B;
  B -- No ----> E[End];
```
