variable "BAKE_VERSION" {
    default = "latest"
}

variable "BAKE_REGISTRY" {
    default = ""
}

variable "BAKE_RELEASENAME" {
    default = ""
}

variable "BAKE_CACHEFROM_REGISTRY" {
    default = ""
}

variable "BAKE_CACHETO_REGISTRY" {
    default = ""
}

variable "BAKE_CACHEFROM_NAME" {
    default = ""
}

variable "BAKE_CACHETO_NAME" {
    default = ""
}

/*
 * Groups for target ordering
 */
group "stage1" {
    targets = ["starling-simple-offboard"]
}

// This target depends on starling-controller-base
target "starling-simple-offboard" {
    context = "."
    args = {
        "VERSION": "${BAKE_VERSION}",
        "REGISTRY": "${BAKE_REGISTRY}"
        }
    tags = [
        "${BAKE_REGISTRY}uobflightlabstarling/starling-simple-offboard:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/starling-simple-offboard:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64", "linux/arm64"]
    cache-to = [ notequal("",BAKE_CACHETO_NAME) ? "${BAKE_CACHETO_REGISTRY}uobflightlabstarling/starling-simple-offboard:${BAKE_CACHETO_NAME}" : "" ]
    cache-from = [ notequal("",BAKE_CACHEFROM_NAME) ? "${BAKE_CACHEFROM_REGISTRY}uobflightlabstarling/starling-simple-offboard:${BAKE_CACHEFROM_NAME}" : "" ]
}