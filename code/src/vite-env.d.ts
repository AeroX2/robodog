/// <reference types="vite/client" />

declare module 'urdf-loader' {
  import { LoadingManager, Object3D } from 'three'

  export default class URDFLoader {
    constructor(manager?: LoadingManager)
    packages: string | Record<string, string>
    load(
      url: string,
      onLoad: (robot: Object3D & { joints: Record<string, Object3D> }) => void,
      onProgress?: (event: ProgressEvent) => void,
      onError?: (error: unknown) => void,
    ): void
  }
}
