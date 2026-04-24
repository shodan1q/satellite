/**
 * Shape sent over the bridge to the Three.js workshop viewer.
 */

export class ViewerOffset {
  x: number = 0;
  y: number = 0;
  z: number = 0;
}

export class ViewerPart {
  id: string = '';
  kind: string = '';
  display: string = '';
  color: string = '#FFFFFF';
  accent: string = '#2E6BF6';
  height: number = 1;
  radiusTop: number = 0.4;
  radiusBottom: number = 0.4;
  explodeOffset: ViewerOffset = new ViewerOffset();
}
