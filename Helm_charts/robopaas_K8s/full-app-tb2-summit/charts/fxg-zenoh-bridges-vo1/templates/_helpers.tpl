{{- define "fxg-zenoh-bridges-vo1.labels" -}}
app.kubernetes.io/name: {{ $.Chart.Name }}
app.kubernetes.io/instance: {{ $.Release.Name }}
app.kubernetes.io/version: {{ $.Chart.Version }}
app.kubernetes.io/component: {{ $.Values.fullnameOverride | default $.Chart.Name }}
{{- end -}}

{{- define "fxg-zenoh-bridges-vo1.selectorLabels" -}}
app.kubernetes.io/name: {{ $.Chart.Name }}
app.kubernetes.io/instance: {{ $.Release.Name }}
{{- end -}}

