{{/*
Expand the name of the chart.
*/}}
{{- define "zenoh.name" -}}
{{ .Chart.Name }}
{{- end }}

{{/*
Create a default fully qualified app name.
*/}}
{{- define "zenoh.fullname" -}}
{{ .Release.Name }}-{{ include "zenoh.name" . }}
{{- end }}

{{/*
Common labels for all resources.
*/}}
{{- define "zenoh.labels" -}}
app: {{ include "zenoh.name" . }}
chart: {{ .Chart.Name }}-{{ .Chart.Version | replace "+" "_" }}
release: {{ .Release.Name }}
heritage: {{ .Release.Service }}
{{- end }}

{{/*
Selector labels for Deployment and Service.
*/}}
{{- define "zenoh.selectorLabels" -}}
app: {{ include "zenoh.name" . }}
release: {{ .Release.Name }}
{{- end }}

{{/*
Create a common namespace reference if set.
*/}}
{{- define "zenoh.namespace" -}}
{{ .Release.Namespace | default "default" }}
{{- end }}

{{/*
Define a common container image reference.
*/}}
{{- define "zenoh.image" -}}
{{ .Values.image.repository }}:{{ .Values.image.tag }}
{{- end }}

